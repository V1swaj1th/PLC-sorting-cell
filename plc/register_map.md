# Register map (v3) - Modbus TCP, OpenPLC slave, port 5020, unit id 1

Only coils are used. The two ranges are disjoint, so the PLC scan never overwrites what ROS wrote.

PLC -> ROS (ladder writes, bridge reads coils 0..11): see the first table in ladder_spec.md
(run=1, estop_active=2, reset_cmd=3, st_idle=4, st_run=5, st_pick=6, st_place=7, st_done=8,
estop_latch=9, st_fault=10, drive_alarm=11; coil 0 = lamp_out from the first test program, ignored).

ROS -> PLC (bridge writes coils 16..26 in one request, ladder only reads them): second table in
ladder_spec.md (start_pb=16, stop_pb=17, estop_ok=18, reset_pb=19, at_home=20, cycle_done=21,
r_fault=22, ph_pick=23, ph_place=24, arm_busy=25, drive_fault=26).

Coil address = 8 * byte + bit of the %QX location (%QX2.4 -> 8*2+4 = 20).

Notes
- estop_ok is a normally-closed loop: 1 = healthy. At power-up it is 0 until the bridge connects, so
  the cell starts in E-STOP and needs a reset, like a real cell.
- Push buttons are momentary: the bridge holds start/stop/reset high for 0.5 s per press.
- drive_fault (26) comes from the servo drive node (`drive/fault`): following-error trip. The PLC
  latches it as drive_alarm (11) until a reset with the fault gone. v3 added these two coils.
- Superseded: the first draft used holding registers %MW for ROS -> PLC. Coils avoid the GT-block
  conversion rungs; the verification step plc_check.py proves the write path.
