# CLAUDE.md - Sabertooth Motor Controller Development Guide

This file provides context for Claude Code (and human developers) working on this package.

## Project Overview

ROS2 Humble Python package that controls Sabertooth 2x16/2x32 motor controllers via PCA9685 PWM for tank-drive robots. Safety-critical - used at conventions around children.

## Quick Start

```bash
# Build:
cd /path/to/motor_controller
colcon build
source install/setup.bash

# Run (simulation mode for Mac):
ros2 launch sabertooth_motor_controller motor_controller.launch.py simulation_mode:=true

# Run tests:
cd src/sabertooth_motor_controller
pytest test/ -v

# Teleop test:
ros2 launch sabertooth_motor_controller teleop.launch.py simulation_mode:=true
```

## Package Structure

```
src/
├── sabertooth_motor_controller_msgs/     # ament_cmake - message definitions
│   └── msg/
│       ├── MotorCommand.msg              # Direct motor control
│       └── MotorStatus.msg               # Status/diagnostics
│
└── sabertooth_motor_controller/          # ament_python - all code
    ├── sabertooth_motor_controller/
    │   ├── motor_controller_node.py      # Main ROS2 node (orchestrator)
    │   ├── pca9685_driver.py             # I2C hardware abstraction
    │   ├── sabertooth_driver.py          # Speed-to-PWM translation
    │   ├── drive_kinematics.py           # Tank drive kinematics
    │   ├── rc_input.py                   # RC receiver GPIO reading
    │   ├── safety_monitor.py             # Safety state machine (CRITICAL)
    │   └── mock_hardware.py              # Mock drivers for testing
    ├── config/
    │   ├── default_params.yaml           # Default configuration
    │   └── sabertooth_2x32_params.yaml   # Override for larger robot
    ├── launch/
    │   ├── motor_controller.launch.py    # Main launch file
    │   └── teleop.launch.py             # Launch with keyboard teleop
    └── test/
        ├── test_kinematics.py
        ├── test_safety.py
        ├── test_sabertooth_driver.py
        └── test_pca9685_driver.py
```

## Key Design Decisions

### Two-Package Architecture
ROS2 requires custom messages in a separate ament_cmake package. `sabertooth_motor_controller_msgs` must be built first.

### Safety-First Design
`safety_monitor.py` is the most critical module. It enforces:
- Command timeout (motors stop if no commands received)
- Per-mode speed limits (teleop: 60%, autonomous: 40%, RC: 100%)
- Acceleration/deceleration rate limiting
- State machine preventing motor output in unsafe states
- E-stop that forces neutral on every control cycle

**INVARIANT: In ESTOP, TIMEOUT, INITIALIZING, ERROR states, motors are ALWAYS neutral. This is checked every 50Hz cycle, not just on state transition.**

### Dependency Injection
All hardware drivers are injected into higher-level modules. `MockPWMDriver` and `MockRCInput` replace real hardware for testing/Mac development. The node auto-detects platform and falls back to mocks.

### PWM Math
PCA9685 at 50Hz, 16-bit duty cycle:
- Neutral (1500us): `duty_cycle = 65535 * 1500/20000 = 4915`
- Full forward (2000us): `duty_cycle = 65535 * 2000/20000 = 6553`
- Full reverse (1000us): `duty_cycle = 65535 * 1000/20000 = 3277`

### Command Priority
`E-STOP > RC Override > Teleop > Autonomous`

RC always wins when active. E-stop blocks everything.

## Module Dependency Graph

```
motor_controller_node.py
├── drive_kinematics.py      (pure math, no deps)
├── safety_monitor.py        (pure Python, no deps)
├── sabertooth_driver.py
│   └── pca9685_driver.py    (or mock_hardware.MockPWMDriver)
│       └── adafruit_pca9685 (hardware only)
├── rc_input.py              (or mock_hardware.MockRCInput)
│   └── Jetson.GPIO          (hardware only)
└── mock_hardware.py         (no deps, used on Mac/CI)
```

## How to Modify

### Adding a motor channel (head/arms)
1. Add channel param in `default_params.yaml`
2. Add channel setup in `motor_controller_node._initialize_hardware()`
3. Create a new topic/service for the actuator
4. Use `self._sabertooth._pwm.set_duty_cycle(channel, duty_cycle)` for direct control

### Changing drive type
1. Implement `DriveKinematics` protocol in `drive_kinematics.py`
2. Add to `create_kinematics()` factory
3. Set `kinematics.type` parameter

### Adjusting safety limits
Edit `config/default_params.yaml` `safety:` section. All limits are ROS2 parameters that can also be set via launch arguments.

## Hardware Notes

- **PCA9685**: I2C bus 1 (or 7), address 0x40, frequency MUST be 50Hz
- **Sabertooth DIP**: 1:OFF 2:ON 3:ON 4:OFF 5:UP 6:UP
- **RC GPIO**: Pins 18, 22, 13 (BOARD numbering). Uses edge detection + timing since Jetson.GPIO has no PWM input support.
- **Jetson I2C**: May need `sudo usermod -aG i2c $USER` and reboot

## Testing Philosophy

- Core modules (kinematics, safety, sabertooth_driver) have no ROS2 deps and are unit-tested with pytest
- MockPWMDriver records all duty cycles for verification
- MockRCInput allows injecting simulated RC pulses
- Safety tests are the most important - they verify the safety invariants
- Run `pytest test/ -v` before any deployment
