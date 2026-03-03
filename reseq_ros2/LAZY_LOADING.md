# Lazy-Loading Node Lifecycle Management

## Overview

The system implements **on-demand (lazy-loading) lifecycle management** for resource-intensive nodes like the thermal camera and LiDAR. Instead of starting all sensing nodes when `app_gateway` launches, they are only started when the Flutter app explicitly requests them.

## Architecture

```
┌─────────────────────┐
│   Flutter App       │
│  (Mobile Client)    │
└──────────┬──────────┘
           │
           │ ROS2 Services
           ↓
┌─────────────────────┐
│   app_gateway       │
│  (Service Router)   │
└──────────┬──────────┘
           │
           ├─→ [ON-DEMAND] thermal_node (thermal camera)
           ├─→ [ON-DEMAND] rplidar_mode (LiDAR controller)
           └─→ [ALWAYS RUNNING] velocity node
```

### Key Features

1. **Lazy Loading**: Nodes are started ONLY when the app requests them
2. **Resource Optimization**: Unused sensors don't consume CPU/GPU/I2C resources
3. **State Tracking**: System maintains node state (RUNNING/STOPPED)
4. **Clean Lifecycle**: Graceful startup and shutdown with proper timeouts
5. **Dynamic Control**: App can toggle sensors on/off at runtime

## Services

### `/ui/thermal` (SetBool)
- **Request**: `data: bool` (true = ON, false = OFF)
- **Response**: `success: bool`, `message: string`
- **Behavior**: 
  - `true` → Starts `thermal_node` (first call only), then requests activation
  - `false` → Requests deactivation from already-running node

### `/ui/lidar` (SetBool)
- **Request**: `data: bool` (true = ON, false = OFF)
- **Response**: `success: bool`, `message: string`
- **Behavior**:
  - `true` → Starts `rplidar_mode` (first call only), then requests activation
  - `false` → Requests deactivation from already-running node

### `/ui/node_status` (NodeStatus)
- **Request**: (empty)
- **Response**: `status: string` (multiline node status report)
- **Example Response**:
  ```
  thermal: STOPPED
  lidar: STOPPED
  velocity: RUNNING
  ```

## Usage Examples

### Start Thermal Camera
```bash
ros2 service call /ui/thermal std_srvs/srv/SetBool "{data: true}"
```
- First call: Launches `thermal_node` + waits 2 seconds for startup
- Service call: Routes to `/activate_thermal` 
- Result: Thermal camera starts publishing images to `/thermal` topic

### Stop Thermal Camera
```bash
ros2 service call /ui/thermal std_srvs/srv/SetBool "{data: false}"
```
- Service call: Routes to `/activate_thermal`
- Result: Thermal camera stops but node remains running (for fast re-enable)

### Query Node Status
```bash
ros2 service call /ui/node_status reseq_interfaces/srv/NodeStatus "{}"
```
- Returns current state of all managed nodes

### Start LiDAR
```bash
ros2 service call /ui/lidar std_srvs/srv/SetBool "{data: true}"
```
- First call: Launches `rplidar_mode` + waits 2 seconds for startup
- Service call: Routes to `/rplidar_mode/toggle_scan`
- Result: LiDAR starts scanning

## Implementation Details

### app_gateway.py

**`_ensure_node_running(module_id)`**
- Checks if node process is alive
- If not running: Launches via subprocess with process group isolation
- Waits 2 seconds for node to register its services
- Returns success/error message

**`_stop_node(module_id)`**
- Terminates running node process
- Sends SIGTERM, waits 3 seconds
- Falls back to SIGKILL if needed
- Cleans up process tracking

**`universal_callback(request, response, module_id)`**
- Intercepts UI service calls
- For ON requests: Ensures node is running
- Waits for hardware service to be ready
- Forwards request with 10-second timeout
- Returns hardware response to app

**`handle_node_status(request, response)`**
- Checks each node's process state
- Returns status string for app display
- Real-time query of current system state

### thermal_node.py

**Lazy Hardware Initialization**
```python
def __init__(self):
    self.is_active = False  # Start disabled
    self.hardware_initialized = False
    self._initialize_hardware()  # Try init, continue if fails
    
    self.srv = self.create_service(SetBool, 'activate_thermal', ...)
```

**Hardware Error Handling**
- I2C/sensor errors don't crash the node
- Node still registers service even if hardware unavailable
- `handle_toggle_camera()` checks `hardware_initialized` flag
- Graceful failure messages to app

**Conditional Publishing**
```python
def publish_thermal_image(self):
    if not self.is_active:  # Don't publish if off
        return
    if not self.hardware_initialized:  # Check hardware state
        return
    # ... actually read and publish
```

### rplidar_node.py

**Subprocess Management**
```python
LAUNCH_CMD = ['ros2', 'launch', 'rplidar_ros', 'rplidar_a2m8_launch.py']

def handle_toggle(self, request, response):
    if request.data:
        response.success, response.message = self.start_rplidar()
    else:
        response.success, response.message = self.stop_rplidar()
    return response
```

**Process Lifecycle**
- `start_rplidar()`: Spawns subprocess, waits 0.5s, checks for early exit
- `stop_rplidar()`: SIGTERM → 3s wait → SIGKILL → 2s wait
- Prevents zombie processes

## Timing Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `_ensure_node_running() sleep` | 2.0s | Allow node time to register its ROS services |
| `universal_callback() retry count` | 5 | Attempts to wait for service readiness |
| `universal_callback() retry delay` | 0.5s | Delay between retry attempts |
| `universal_callback() RPC timeout` | 10.0s | Total time to wait for hardware service response |
| `start_rplidar() verify delay` | 0.5s | Check process didn't die immediately |
| `stop_rplidar() SIGTERM timeout` | 3.0s | Graceful shutdown wait |
| `stop_rplidar() SIGKILL timeout` | 2.0s | Force kill wait |

## Resource Usage Comparison

### Before (Always Running)
- thermal_node: ~5% CPU (I2C polling)
- rplidar_mode: ~3% CPU + 1 USB connection
- **Total**: ~8% CPU always

### After (Lazy Loading)
- Idle state: 0% CPU for unused sensors
- Thermal ON: ~5% CPU only when needed
- LiDAR ON: ~3% CPU only when needed
- **Savings**: 100% when sensors disabled

## Error Handling

### Node Fails to Start
- `universal_callback()` returns `success=False`
- Message describes the error
- App receives error and can retry

### Service Not Ready in Time
- Retry loop attempts 5 times with 0.5s delays
- Total wait: up to 10 seconds
- If still not ready: Timeout error
- Suggests node failed to initialize

### Hardware Unavailable
- thermal_node continues running but disabled
- Accepts toggle requests but returns error
- App can detect and show "Hardware Not Available" message
- Device remains operational for other functions

## Future Improvements

- [ ] Automatic node restart on unexpected exit
- [ ] Publish node heartbeat topics for app monitoring
- [ ] Implement node health check service
- [ ] Support for priority-based node startup ordering
- [ ] Configurable timeout values via ROS2 parameters
- [ ] Node startup failure notifications to app

## Testing Commands

```bash
# Start app_gateway
ros2 run reseq_ros2 app_gateway

# In another terminal:

# Check initial state (all stopped)
ros2 service call /ui/node_status reseq_interfaces/srv/NodeStatus "{}"

# Turn on thermal camera (should auto-start thermal_node)
ros2 service call /ui/thermal std_srvs/srv/SetBool "{data: true}"

# Check status again (thermal should be RUNNING)
ros2 service call /ui/node_status reseq_interfaces/srv/NodeStatus "{}"

# Turn off thermal camera (node keeps running, just disables publishing)
ros2 service call /ui/thermal std_srvs/srv/SetBool "{data: false}"

# Turn on LiDAR (should auto-start rplidar_mode)
ros2 service call /ui/lidar std_srvs/srv/SetBool "{data: true}"

# List services to verify all are available
ros2 service list | grep -E "^/ui/"
```
