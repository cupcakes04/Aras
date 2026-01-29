import asyncio
import random
import time
from dataclasses import dataclass
from typing import Dict, List, Any, Optional
import json

# ==================== Mock Hardware Classes ====================
class MockPin:
    """Mock hardware pin that can be read/written"""
    def __init__(self, pin_number: int, initial_value: float = 0.0):
        self.pin_number = pin_number
        self.value = initial_value
        self.mode = "input"  # or "output"
    
    def read(self) -> float:
        """Simulate reading from pin with some noise"""
        if self.mode == "output":
            return self.value
        # Add some noise for realism
        noise = random.uniform(-0.1, 0.1)
        return self.value + noise
    
    def write(self, value: float):
        """Simulate writing to pin"""
        self.value = value
        self.mode = "output"
        # Simulate small write delay
        time.sleep(0.0001)
    
    def __repr__(self):
        return f"Pin({self.pin_number}): {self.value:.2f}"

class MockSensor:
    """Base mock sensor class"""
    def __init__(self, sensor_id: str, pins: List[MockPin]):
        self.sensor_id = sensor_id
        self.pins = pins
        self.last_read_time = 0
        
    async def read_async(self):
        """Async read with simulated I/O delay"""
        await asyncio.sleep(0.001)  # Simulate I/O delay
        readings = {}
        for i, pin in enumerate(self.pins):
            readings[f"pin_{i}"] = pin.read()
        self.last_read_time = time.time()
        return readings
    
    async def write_async(self, pin_index: int, value: float):
        """Async write with simulated I/O delay"""
        await asyncio.sleep(0.001)  # Simulate I/O delay
        if 0 <= pin_index < len(self.pins):
            self.pins[pin_index].write(value)
            return True
        return False

# ==================== Sensor Type 1: Simple Read/Write ====================
class Type1Sensor(MockSensor):
    """Sensor Type 1: Simple read/write without external variables"""
    
    def __init__(self, sensor_id: str, pins: List[MockPin]):
        super().__init__(sensor_id, pins)
        self.history = []  # Local history, not shared
        
    async def read_and_process(self):
        """Read pins and apply simple processing"""
        readings = await self.read_async()
        
        # Simple processing: average and threshold check
        values = list(readings.values())
        avg_value = sum(values) / len(values) if values else 0
        
        # Apply threshold logic
        if avg_value > 2.5:
            # Write to output pin (pin 0 is output in this example)
            await self.write_async(0, 3.3)  # Set to max
        else:
            await self.write_async(0, 0.0)  # Set to min
            
        # Store in local history
        self.history.append({
            'timestamp': time.time(),
            'readings': readings,
            'average': avg_value
        })
        
        # Keep only last 100 readings
        if len(self.history) > 100:
            self.history.pop(0)
            
        return {
            'sensor_id': self.sensor_id,
            'readings': readings,
            'average': avg_value,
            'action': 'high' if avg_value > 2.5 else 'low'
        }

# ==================== Sensor Type 2: Heavy Processing ====================
class Type2Sensor(MockSensor):
    """Sensor Type 2: Read then run heavy model (simulated with delay)"""
    
    def __init__(self, sensor_id: str, pins: List[MockPin], model_name: str = "default"):
        super().__init__(sensor_id, pins)
        self.model_name = model_name
        self.inference_count = 0
        
    async def run_model_inference(self, data: Dict):
        """Simulate heavy model inference (1 second delay)"""
        # Simulate model loading/processing time
        await asyncio.sleep(1.0)  # This is the heavy processing part
        
        # Mock "AI model" processing - could be ML inference
        values = list(data.values())
        avg = sum(values) / len(values) if values else 0
        
        # Simulate complex model output
        prediction = {
            'anomaly_score': abs(avg - 1.5) * 100,  # Distance from ideal
            'trend': 'increasing' if avg > 1.5 else 'decreasing',
            'confidence': min(95, self.inference_count % 100),
            'model': self.model_name
        }
        
        self.inference_count += 1
        return prediction
    
    async def read_and_analyze(self):
        """Read pins and run heavy analysis"""
        # Step 1: Read sensor data
        readings = await self.read_async()
        
        # Step 2: Run heavy model (this will take ~1 second)
        # We run this in a separate thread to not block the event loop
        loop = asyncio.get_event_loop()
        model_result = await loop.run_in_executor(
            None,  # Uses default ThreadPoolExecutor
            lambda: asyncio.run(self.run_model_inference(readings))
        )
        
        # Step 3: Based on model result, potentially write to pins
        if model_result['anomaly_score'] > 50:
            # High anomaly - trigger output pin
            await self.write_async(0, 3.3)  # Alarm signal
        
        return {
            'sensor_id': self.sensor_id,
            'readings': readings,
            'model_result': model_result,
            'has_anomaly': model_result['anomaly_score'] > 50
        }

# ==================== Sensor Type 3: Read/Write with External Variables ====================
class Type3Sensor(MockSensor):
    """Sensor Type 3: Read/write with access to external shared variables"""
    
    def __init__(self, sensor_id: str, pins: List[MockPin], 
                 shared_state: Dict[str, Any], control_params: Dict[str, float]):
        super().__init__(sensor_id, pins)
        self.shared_state = shared_state  # Reference to external dict
        self.control_params = control_params
        self.setpoint = control_params.get('setpoint', 1.0)
        self.kp = control_params.get('kp', 0.5)  # Proportional gain
        
    async def read_and_control(self):
        """Read pins and implement control logic using external variables"""
        readings = await self.read_async()
        
        # Calculate average of readings
        values = list(readings.values())
        process_value = sum(values) / len(values) if values else 0
        
        # PID-like control using external setpoint
        error = self.setpoint - process_value
        control_output = self.kp * error
        
        # Clamp output
        control_output = max(0.0, min(3.3, control_output))
        
        # Write control output to first pin
        await self.write_async(0, control_output)
        
        # Update shared state
        self.shared_state.update({
            f'{self.sensor_id}_pv': process_value,  # Process variable
            f'{self.sensor_id}_error': error,
            f'{self.sensor_id}_output': control_output,
            f'{self.sensor_id}_timestamp': time.time()
        })
        
        # Also update global control parameters if needed
        if 'global_setpoint' in self.shared_state:
            self.setpoint = self.shared_state['global_setpoint']
        
        return {
            'sensor_id': self.sensor_id,
            'process_value': process_value,
            'setpoint': self.setpoint,
            'control_output': control_output,
            'error': error
        }

# ==================== Global State Manager ====================
class GlobalState:
    """Manages global state shared between sensors"""
    def __init__(self):
        self.data = {}
        self.lock = asyncio.Lock()
    
    async def update(self, key: str, value: Any):
        """Thread-safe update"""
        async with self.lock:
            self.data[key] = value
    
    async def get(self, key: str, default: Any = None):
        """Thread-safe get"""
        async with self.lock:
            return self.data.get(key, default)
    
    def to_dict(self):
        """Get copy of current state"""
        return self.data.copy()

# ==================== Main System ====================
class SensorControlSystem:
    """Main control system that manages all sensors"""
    
    def __init__(self):
        self.global_state = GlobalState()
        self.sensors = []
        self.results_queue = asyncio.Queue()
        self.running = False
        
        # Initialize with some default values
        self.global_state.data = {
            'global_setpoint': 1.5,
            'system_mode': 'normal',
            'emergency_stop': False,
            'total_readings': 0
        }
    
    def initialize_sensors(self):
        """Create and configure sensor instances"""
        # Create mock pins
        pins_type1 = [MockPin(i) for i in range(3)]
        pins_type2 = [MockPin(i) for i in range(2)]
        pins_type3 = [MockPin(i) for i in range(4)]
        
        # Initialize with some values
        for pin in pins_type1 + pins_type2 + pins_type3:
            pin.value = random.uniform(0, 3.3)
        
        # Create sensor instances
        sensor1 = Type1Sensor("temp_sensor_1", pins_type1)
        sensor2 = Type2Sensor("vibration_sensor", pins_type2, model_name="vibration_model")
        sensor3 = Type3Sensor("pressure_ctrl", pins_type3, 
                             self.global_state.data,  # Shared state reference
                             {'setpoint': 1.5, 'kp': 0.8})
        
        self.sensors = [
            {'sensor': sensor1, 'type': 'type1', 'interval_ms': 100},   # 10 Hz
            {'sensor': sensor2, 'type': 'type2', 'interval_ms': 2000},  # 0.5 Hz (slow due to heavy processing)
            {'sensor': sensor3, 'type': 'type3', 'interval_ms': 50},    # 20 Hz
        ]
    
    async def sensor_worker(self, sensor_info: Dict, interval_ms: int):
        """Worker that reads and processes a sensor at fixed interval"""
        sensor = sensor_info['sensor']
        sensor_type = sensor_info['type']
        
        print(f"Starting {sensor_type} worker for {sensor.sensor_id} at {interval_ms}ms interval")
        
        while self.running:
            try:
                start_time = time.time()
                
                # Process based on sensor type
                if sensor_type == 'type1':
                    result = await sensor.read_and_process()
                elif sensor_type == 'type2':
                    result = await sensor.read_and_analyze()
                elif sensor_type == 'type3':
                    result = await sensor.read_and_control()
                else:
                    result = {'error': 'unknown sensor type'}
                
                # Add timing info
                result['processing_time'] = time.time() - start_time
                result['timestamp'] = time.time()
                
                # Put result in queue for monitoring
                await self.results_queue.put(result)
                
                # Update global statistics
                await self.global_state.update('total_readings', 
                    self.global_state.data.get('total_readings', 0) + 1)
                
                # Wait for next cycle
                elapsed = time.time() - start_time
                sleep_time = max(0, interval_ms / 1000 - elapsed)
                await asyncio.sleep(sleep_time)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error in sensor worker {sensor.sensor_id}: {e}")
                await asyncio.sleep(1)  # Prevent tight error loop
    
    async def monitor_task(self):
        """Monitor task that processes results from all sensors"""
        print("Starting monitor task...")
        while self.running:
            try:
                # Get result with timeout
                result = await asyncio.wait_for(self.results_queue.get(), timeout=1.0)
                
                # Process result based on sensor type
                sensor_id = result.get('sensor_id', 'unknown')
                
                if 'has_anomaly' in result and result['has_anomaly']:
                    print(f"⚠️  ANOMALY DETECTED by {sensor_id}")
                    # Could trigger global actions here
                    await self.global_state.update('last_anomaly', {
                        'sensor': sensor_id,
                        'time': time.time(),
                        'data': result
                    })
                
                # Print summary periodically
                if self.global_state.data.get('total_readings', 0) % 10 == 0:
                    self.print_system_status()
                    
            except asyncio.TimeoutError:
                continue  # No data, continue
            except Exception as e:
                print(f"Error in monitor: {e}")
    
    async def command_listener(self):
        """Task to handle external commands (e.g., change setpoints)"""
        print("Starting command listener...")
        while self.running:
            try:
                # Simulate receiving commands (in real system, this could be from network)
                await asyncio.sleep(5)  # Check for commands every 5 seconds
                
                # Example: Toggle setpoint every 10 seconds
                current_time = time.time()
                if int(current_time) % 10 < 5:
                    new_setpoint = 1.5
                else:
                    new_setpoint = 2.0
                
                if self.global_state.data.get('global_setpoint', 0) != new_setpoint:
                    await self.global_state.update('global_setpoint', new_setpoint)
                    print(f"Command: Setpoint changed to {new_setpoint}")
                    
            except asyncio.CancelledError:
                break
    
    def print_system_status(self):
        """Print current system status"""
        print("\n" + "="*50)
        print(f"System Status at {time.strftime('%H:%M:%S')}")
        print(f"Total readings: {self.global_state.data.get('total_readings', 0)}")
        print(f"Global setpoint: {self.global_state.data.get('global_setpoint', 'N/A')}")
        print(f"Queue size: {self.results_queue.qsize()}")
        
        # Print last result from each sensor type
        print("\nLast known states:")
        for sensor_info in self.sensors:
            sensor = sensor_info['sensor']
            if hasattr(sensor, 'history') and sensor.history:
                last = sensor.history[-1]
                print(f"  {sensor.sensor_id}: avg={last.get('average', 'N/A'):.2f}")
        print("="*50 + "\n")
    
    async def run_system(self, duration_seconds: int = 30):
        """Run the entire system for specified duration"""
        self.running = True
        self.initialize_sensors()
        
        # Start all sensor workers
        worker_tasks = []
        for sensor_info in self.sensors:
            task = asyncio.create_task(
                self.sensor_worker(sensor_info, sensor_info['interval_ms'])
            )
            worker_tasks.append(task)
        
        # Start monitoring and command tasks
        monitor_task_obj = asyncio.create_task(self.monitor_task())
        command_task = asyncio.create_task(self.command_listener())
        
        print(f"\n{'='*60}")
        print("Sensor Control System Started")
        print(f"Running for {duration_seconds} seconds")
        print(f"Monitoring {len(self.sensors)} sensors")
        print(f"{'='*60}\n")
        
        # Run for specified duration
        await asyncio.sleep(duration_seconds)
        
        # Shutdown
        print("\nShutting down system...")
        self.running = False
        
        # Cancel all tasks
        for task in worker_tasks:
            task.cancel()
        monitor_task_obj.cancel()
        command_task.cancel()
        
        # Wait for tasks to finish
        await asyncio.gather(*worker_tasks, monitor_task_obj, command_task, 
                           return_exceptions=True)
        
        # Final status
        self.print_system_status()
        print("\nSystem shutdown complete.")

# ==================== Main Entry Point ====================
async def main():
    """Main entry point"""
    system = SensorControlSystem()
    
    # Run system for 30 seconds (Ctrl+C to interrupt earlier)
    try:
        await system.run_system(duration_seconds=30)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        system.running = False

if __name__ == "__main__":
    asyncio.run(main())