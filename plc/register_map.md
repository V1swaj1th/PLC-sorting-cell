# Modbus register map (OpenPLC Runtime v4 slave plugin, 127.0.0.1:5020, 0-based addresses)

Scan cycle: 20 ms. PLC owns %QX/%QW; ROS writes only %MW (holding regs 1024+).

| Direction | Signal | PLC var | Modbus | Status |
|---|---|---|---|---|
| PLC -> ROS | lamp_out (test) | %QX0.0 | coil 0 | verified |
| PLC -> ROS | heartbeat (test) | %QW0 | holding reg 0 | verified |
| ROS -> PLC | mb_cmd (test) | %MW0 | holding reg 1024 | verified |
| PLC -> ROS | run, estop_active, reset_cmd | %QX0.1-0.3 | coils 1-3 | planned |
| PLC -> ROS | cycle_state | %QW1 | holding reg 1 | planned |
| ROS -> PLC | arm_busy, at_home, cycle_done, fault | %MW1-%MW4 | holding regs 1025-1028 | planned |
| Panel -> PLC | start_pb, stop_pb, estop_ok (NC, 1 = healthy) | %MW10-%MW12 | holding regs 1034-1036 | planned |

Notes: %MX coils are disabled by default (mx_bits: 0). Discrete inputs and
input registers are read-only over Modbus, so ROS cannot write them.
After every program upload from the Editor, re-run plc_setup.sh (on the host).
