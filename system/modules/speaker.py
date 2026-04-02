import asyncio
import random
import time
import io
import math
import struct
import subprocess
import wave
from collections import deque
from .history import History

class SpeakerOld(History):
    """Speaker, 
    data in:
        - str of characters (not sure, maybe a list of stuff idk)
        - 0.0~1.0 signal strength, normalised
    (Legacy Dummy)
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def write(self, value: str):
        # 1. Perform the physical action (placeholder)
        
        # 2. Record the action in history
        self.save_history(value)


# ---------------------------------------------------------------------------
# Warning sound identifiers
# ---------------------------------------------------------------------------
WARN_FCW               = "fcw"
WARN_RCW               = "rcw"
WARN_EMERGENCY_BRAKING = "emergency_braking"

# ---------------------------------------------------------------------------
# Tone definitions for each warning
# Each entry is a list of (frequency_hz, duration_seconds) tuples.
# A frequency of 0 produces silence (gap between beeps).
# ---------------------------------------------------------------------------
_WARN_TONES = {
    WARN_FCW: [                         # 3 short fast beeps — urgent
        (1000, 0.15), (0, 0.08),
        (1000, 0.15), (0, 0.08),
        (1000, 0.15),
    ],
    WARN_RCW: [                         # 2 medium beeps — caution
        (800, 0.2), (0, 0.15),
        (800, 0.2),
    ],
    WARN_EMERGENCY_BRAKING: [           # 1 long continuous beep — critical
        (1500, 0.08), (0, 0.08),
        (1500, 0.08), (0, 0.08),
        (1500, 0.08), (0, 0.08),
        (1500, 0.08), (0, 0.08),
        (1500, 0.08), (0, 0.08),
        (1500, 0.08), (0, 0.08),
    ],
}

# WAV parameters
_SAMPLE_RATE  = 44100   # Hz
_AMPLITUDE    = 28000   # 0-32767, kept below max to avoid distortion

class Speaker(History):
    """
    Non-blocking speaker driver for ARAS warning beep tones.
    Generates WAV audio data in memory and pipes it to aplay.
    Integrated into system.py History structure.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup()
        
    def setup(self):
        self._process = None    # currently running aplay subprocess
        self._hardware_available = True
        
        # Check if aplay is available
        try:
            subprocess.run(["aplay", "--version"], capture_output=True, check=True)
        except Exception as e:
            print(f"[Speaker] 'aplay' not found or failed: {e}. Running in dummy mode.")
            self._hardware_available = False
            
        # Pre-generate all warning tones as WAV bytes at startup
        self._wav_cache = {
            warning: self._generate_wav(tones)
            for warning, tones in _WARN_TONES.items()
        }

    def close(self):
        self.stop()

    def play(self, warning: str) -> bool:
        wav_data = self._wav_cache.get(warning)
        if wav_data is None:
            return False

        self.stop()

        if not self._hardware_available:
            return True

        try:
            self._process = subprocess.Popen(
                ["aplay", "-"],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._process.stdin.write(wav_data)
            self._process.stdin.close()
        except Exception as e:
            print(f"[Speaker] Failed to play: {e}")
            return False

        return True

    def stop(self) -> None:
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    @property
    def is_playing(self) -> bool:
        if self._process is None:
            return False
        return self._process.poll() is None

    def _generate_wav(self, tones: list) -> bytes:
        samples = bytearray()
        for frequency, duration in tones:
            num_samples = int(_SAMPLE_RATE * duration)
            for i in range(num_samples):
                if frequency == 0:
                    sample = 0
                else:
                    sample = int(_AMPLITUDE * math.sin(2 * math.pi * frequency * i / _SAMPLE_RATE))
                samples += struct.pack("<h", sample)

        buf = io.BytesIO()
        with wave.open(buf, "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(_SAMPLE_RATE)
            wav.writeframes(bytes(samples))

        return buf.getvalue()

    async def write(self, value: str):
        """
        value: str corresponding to WARN_FCW, WARN_RCW, or WARN_EMERGENCY_BRAKING
        """
        if value:
            self.play(value)
        else:
            self.stop()
        self.save_history(value)

