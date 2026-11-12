# Bidirectional diagnostics

`bidirectional_energy_flow_control.launch` starts a recorder. When the controller is switched to running, it creates a CSV for each activation in `$ROS_HOME/bidirectional_logs/` (default `~/.ros/bidirectional_logs/`). The recorder prints the full path. The recorder must be running for logs to be saved; starting the controller by other means requires starting `record_bidirectional_diagnostics.py` separately.

The controller publishes `/bidirectional_energy_flow_control/diagnostics` with nonblocking `RealtimePublisher::trylock`. The recorder performs file I/O outside the control loop. Recording is best effort: `cycle` gaps identify lost samples, while `publish_drops` counts failed publisher locks, not subscriber or disk loss. Sudden process or power loss can lose buffered records. The recorder does not control or stop the robot.

Columns are one-based for arrays. `q`, `dq` and torque fields are joint order; `wrench` is the last signed sensor sample in the configured input frame, before the controller rotates it into the base frame. `calculated_torque` is before limiting; `commanded_torque` is after limiting. `previous_torque` is Franka's previous desired torque. `wrench_age` is seconds (`-1` if invalid); `wrench_fresh` uses the configured timeout. `period` is in seconds.

`status`: 0 normal; 1 wrench stale/invalid; 2 invalid state/model or algorithm initialization; 3 invalid target; 4 invalid algorithm output; 5 return routine. When `diagnostics_valid` is false, nominal, calculated and energy fields are retained from the last valid calculation and must not be interpreted as current. Diagnostics do not establish controller safety or guarantee that every control cycle is captured.
