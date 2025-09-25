# Motor ID Configuration Guide

The firmware expects every DM-series actuator to use a fixed pair of CAN
identifiers: one for outbound MIT commands and one for inbound feedback.  The
mapping is defined in `User/Devices/DM_Motor/dm4310_drv.c` and is summarised
below for convenience.  Ensure that each motor on the bus is programmed with
**both** identifiers so that the control board can communicate with it.

| Joint | Command ID | Feedback ID | CAN Bus |
| ----- | ---------- | ----------- | ------- |
| waist_yaw | 0x09 | 0x19 | FDCAN3 |
| neck_yaw | 0x0A | 0x1A | FDCAN3 |
| neck_pitch | 0x0B | 0x1B | FDCAN3 |
| neck_roll | 0x0C | 0x1C | FDCAN3 |
| left_shoulder_pitch | 0x41 | 0x51 | FDCAN2 |
| left_shoulder_roll | 0x42 | 0x52 | FDCAN2 |
| left_elbow | 0x43 | 0x53 | FDCAN2 |
| left_wrist | 0x44 | 0x54 | FDCAN2 |
| right_shoulder_pitch | 0x21 | 0x31 | FDCAN1 |
| right_shoulder_roll | 0x22 | 0x32 | FDCAN1 |
| right_elbow | 0x23 | 0x33 | FDCAN1 |
| right_wrist | 0x24 | 0x34 | FDCAN1 |
| left_hip_pitch | 0x01 | 0x15 | FDCAN2 |
| left_hip_yaw | 0x02 | 0x14 | FDCAN2 |
| left_hip_roll | 0x03 | 0x13 | FDCAN2 |
| left_knee | 0x04 | 0x12 | FDCAN2 |
| left_ankle_pitch | 0x05 | 0x11 | FDCAN2 |
| left_ankle_roll | 0x06 | 0x16 | FDCAN2 |
| left_toe | 0x07 | 0x17 | FDCAN2 |
| right_hip_pitch | 0x01 | 0x11 | FDCAN1 |
| right_hip_yaw | 0x0A | 0x1A | FDCAN1 |
| right_hip_roll | 0x0B | 0x12 | FDCAN1 |
| right_knee | 0x05 | 0x15 | FDCAN1 |
| right_ankle_pitch | 0x07 | 0x17 | FDCAN1 |
| right_ankle_roll | 0x08 | 0x18 | FDCAN1 |
| right_toe | 0x09 | 0x19 | FDCAN1 |

## Programming the IDs

1. Power the motor from a bench supply and connect the vendor CAN dongle.
2. Use the manufacturer configuration tool (or a simple MIT command frame) to
   assign the command identifier from the table above.
3. Set the feedback identifier to the matching value from the table.
4. Repeat the procedure for every actuator connected to the same CAN bus.
5. Power-cycle the motor to ensure that the new identifiers are stored in
   non-volatile memory.

> **Tip:** Many DM-series actuators keep their configuration only after a clean
> shutdown. Disconnect the supply after programming each leg/arm to avoid
> accidental resets while other motors are still being configured.

Once all motors expose the expected identifiers the firmware will automatically
route feedback frames to the correct joint and apply the default MIT gains from
`control_board_profile.c`.
