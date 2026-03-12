"""
speaker.py — Speaker Library
ARAS (Advanced Rider Assistance System)

Hardware : Speaker connected via 3.5mm audio jack
Interface: ALSA audio via aplay subprocess
Platform : Radxa Cubie A7A

Generates and plays distinct beep tones for each warning type.
No pre-recorded audio files needed — tones are generated in code
as raw WAV data and piped directly to aplay.

Warning sounds:
  FCW — Front Collision Warning    : 3 short fast beeps (urgent)
  RCW — Rear Collision Warning     : 2 medium beeps (caution)
  EB  — Emergency Braking          : 1 long continuous beep (critical)

Dependencies:
  aplay — part of alsa-utils (usually pre-installed on Linux)
  Install if missing: sudo apt install alsa-utils
"""

import io
import math
import struct
import subprocess
import wave


# ---------------------------------------------------------------------------
# Warning sound identifiers — use these constants in main.py
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
        (1000, 0.15),
        (0,    0.08),
        (1000, 0.15),
        (0,    0.08),
        (1000, 0.15),
    ],
    WARN_RCW: [                         # 2 medium beeps — caution
        (800, 0.2),
        (0,   0.15),
        (800, 0.2),
    ],
    WARN_EMERGENCY_BRAKING: [           # 1 long continuous beep — critical
        (1500, 0.08),
        (0,   0.08),
        (1500, 0.08),
        (0,   0.08),
        (1500, 0.08),
        (0,   0.08),
        (1500, 0.08),
        (0,   0.08),
        (1500, 0.08),
        (0,   0.08),
        (1500, 0.08),
        (0,   0.08),
    ],
}

# WAV parameters
_SAMPLE_RATE  = 44100   # Hz
_AMPLITUDE    = 28000   # 0-32767, kept below max to avoid distortion


# ---------------------------------------------------------------------------
# Speaker class
# ---------------------------------------------------------------------------
class Speaker:
    """
    Non-blocking speaker driver for ARAS warning beep tones.

    Generates WAV audio data in memory and pipes it to aplay.
    Each play() call launches aplay in the background so main.py
    is never blocked. If a sound is already playing when play() is
    called, the current sound is stopped and the new one starts immediately.

    Example
    -------
        speaker = Speaker()

        speaker.play(WARN_FCW)               # 3 short fast beeps
        speaker.play(WARN_RCW)               # 2 medium beeps
        speaker.play(WARN_EMERGENCY_BRAKING) # 1 long beep

        speaker.stop()
        speaker.close()
    """

    def __init__(self):
        self._process = None    # currently running aplay subprocess
        # Pre-generate all warning tones as WAV bytes at startup
        self._wav_cache = {
            warning: self._generate_wav(tones)
            for warning, tones in _WARN_TONES.items()
        }

    def close(self):
        """Stop any playing sound and release resources."""
        self.stop()

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------

    def play(self, warning: str) -> bool:
        """
        Play a warning beep tone in the background (non-blocking).

        If a sound is already playing, it is stopped immediately and the
        new sound starts. Returns immediately without waiting for the
        tone to finish.

        Parameters
        ----------
        warning : str
            Warning sound identifier. Use the module-level constants:
              WARN_FCW               — front collision warning (3 short beeps)
              WARN_RCW               — rear collision warning (2 medium beeps)
              WARN_EMERGENCY_BRAKING — emergency braking (1 long beep)

        Returns
        -------
        bool
            True if playback started successfully.
            False if the warning identifier is unknown.
        """
        wav_data = self._wav_cache.get(warning)
        if wav_data is None:
            return False

        # Stop current sound if one is playing
        self.stop()

        # Pipe WAV data directly to aplay stdin — no temp files needed
        self._process = subprocess.Popen(
            ["aplay", "-"],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Write WAV data to stdin and close it (non-blocking)
        self._process.stdin.write(wav_data)
        self._process.stdin.close()

        return True

    def stop(self) -> None:
        """
        Stop the currently playing sound immediately.

        Does nothing if no sound is playing.
        """
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    @property
    def is_playing(self) -> bool:
        """
        True if a sound is currently playing, False otherwise. Read-only.
        """
        if self._process is None:
            return False
        return self._process.poll() is None

    # ------------------------------------------------------------------
    # Private: tone generation
    # ------------------------------------------------------------------

    def _generate_wav(self, tones: list) -> bytes:
        """
        Generate a WAV audio byte string from a list of tone segments.

        Parameters
        ----------
        tones : list of (frequency, duration) tuples
            frequency : int  — Hz. 0 = silence.
            duration  : float — seconds.

        Returns
        -------
        bytes — complete WAV file data ready to pipe to aplay.
        """
        samples = bytearray()

        for frequency, duration in tones:
            num_samples = int(_SAMPLE_RATE * duration)

            for i in range(num_samples):
                if frequency == 0:
                    sample = 0  # silence
                else:
                    # Sine wave sample
                    sample = int(_AMPLITUDE * math.sin(
                        2 * math.pi * frequency * i / _SAMPLE_RATE
                    ))
                # Pack as 16-bit little-endian signed int
                samples += struct.pack("<h", sample)

        # Wrap raw samples in a WAV container
        buf = io.BytesIO()
        with wave.open(buf, "wb") as wav:
            wav.setnchannels(1)           # mono
            wav.setsampwidth(2)           # 16-bit
            wav.setframerate(_SAMPLE_RATE)
            wav.writeframes(bytes(samples))

        return buf.getvalue()