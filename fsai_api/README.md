# fsai_api
Bridges communication between ROS nodes and the Vehicle Control Unit (VCU) via CAN.


## Installation
- Install CAN utilities: `sudo apt install can-utils`
- Optional: `rosdep install --from-paths src -r -y`.

## Usage

### 1. Setup CAN Interface

**For real CAN (can0):**
```bash
sudo ip link set can0 up type can bitrate 500000
```

**For virtual CAN (vcan0) - testing only:**
```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

### 2. Launch CAN Bridge

**Real CAN:**
```bash
ros2 run fsai_api ackermann_can can0
```

**Virtual CAN:**
```bash
ros2 run fsai_api ackermann_can vcan0
```

### 3. Run Speed Controller (Optional)
```bash
ros2 run fsai_api wheel_speed_controller.py
```

### 4. Cleanup (Virtual CAN only)
When done testing with virtual CAN:
```bash
sudo ip link delete vcan0
```

## Interface

| Node | Inputs | Outputs | Description |
|------|--------|---------|-------------|
| `ackermann_can` | `/ackermann_cmd` (`ackermann_msgs/msg/AckermannDrive`)<br>`/emergency_brake` (`std_msgs/msg/Bool`)<br>`/chequered_flag` (`std_msgs/msg/Bool`)<br>`/brake` (`std_msgs/msg/Bool`) | `/vcu2ai` (`fsai_api/msg/VCU2AI`) | ROS↔VCU CAN bridge for drive commands and vehicle state |
| `wheel_speed_controller` | `/vcu2ai` (`fsai_api/msg/VCU2AI`)<br>`/ackermann_cmd_controller` (`ackermann_msgs/msg/AckermannDrive`) | `/ackermann_cmd` (`ackermann_msgs/msg/AckermannDrive`) | PI controller for wheel speed regulation |

## Notes
- **TODO:** Implement a vcan0 VCU state machine simulator based on the ADS-DV


