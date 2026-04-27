# astro_ros_agent_watcher

micro-ROS agent watchdog that monitors heartbeat and auto-restarts the agent on failure.

## Node: `microros_watchdog`

Subscribes to a heartbeat topic (default `/bno055_imu_raw`) with best-effort QoS. If no message arrives within a configurable timeout (default 5 seconds), it kills the `micro_ros_agent` process and restarts it with a cooldown period. Handles UDP port conflicts during restart.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| Heartbeat topic | `/bno055_imu_raw` | Topic to monitor for liveness |
| Timeout | 5s | Seconds before declaring agent dead |
