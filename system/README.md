Here's a complete example with mock sensor classes and a full implementation, hand-coded with deepshit:

## Key Features:

### **1. Three Sensor Types:**
- **Type1**: Simple read/write with local processing
- **Type2**: Heavy processing (simulated with 1s delay) with model inference
- **Type3**: Control system with access to shared global variables

### **2. Global State Management:**
- Shared dictionary accessible by all sensors
- Thread-safe updates using asyncio locks
- Global variables like `global_setpoint` can be modified by any component

### **3. Multi-rate Execution:**
- Each sensor runs at its own frequency (10Hz, 0.5Hz, 20Hz)
- Proper timing with `asyncio.sleep()` accounting for processing time

### **4. Communication Patterns:**
- Results queue for inter-task communication
- Global state for shared variables
- Async/await for non-blocking I/O

### **5. Monitoring and Control:**
- Monitor task processes all sensor outputs
- Command listener for external control
- Periodic system status reporting

### **6. Mock Hardware:**
- `MockPin` class simulates GPIO pins
- Simulated I/O delays and noise
- Can be replaced with real hardware drivers

## To Use:

1. **Run the system:** `python sensor_system.py`
2. **Observe output:** Different sensor types running at different rates
3. **See interactions:** Type3 sensor uses global variables set by command task
4. **Monitor:** System prints status every 10 readings

## Extending:

- Add real hardware drivers by replacing `MockPin` with actual GPIO library
- Implement real ML models in Type2 sensor
- Add network interface for remote monitoring
- Add persistence (database logging)
- Implement fault tolerance and error recovery

The system demonstrates how to manage different timing requirements while maintaining clean architecture and proper inter-task communication.