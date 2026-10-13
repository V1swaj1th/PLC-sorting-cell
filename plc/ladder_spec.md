# Ladder program spec (OpenPLC Editor) - sorting cell

All signals are BOOL. Contact `-| |-` = normal, `-|/|-` = negated. Coil types: ( ) normal, (S) set, (R) reset.
Keep the three test rungs/variables you already have (start_in, mb_cmd, lamp_out). Add the variables and rungs below.

## Variables (Name / Class / Type / Location)

PLC-driven (the ladder writes them):

| Name | Class | Type | Location | Modbus coil |
|---|---|---|---|---|
| run | output | BOOL | %QX0.1 | 1 |
| estop_active | output | BOOL | %QX0.2 | 2 |
| reset_cmd | output | BOOL | %QX0.3 | 3 |
| st_idle | output | BOOL | %QX0.4 | 4 |
| st_run | output | BOOL | %QX0.5 | 5 |
| st_pick | output | BOOL | %QX0.6 | 6 |
| st_place | output | BOOL | %QX0.7 | 7 |
| st_done | output | BOOL | %QX1.0 | 8 |
| estop_latch | output | BOOL | %QX1.1 | 9 |
| st_fault | output | BOOL | %QX1.2 | 10 |

ROS-driven (the bridge writes them; NO rung may ever use them as a coil):

| Name | Class | Type | Location | Modbus coil |
|---|---|---|---|---|
| start_pb | output | BOOL | %QX2.0 | 16 |
| stop_pb | output | BOOL | %QX2.1 | 17 |
| estop_ok | output | BOOL | %QX2.2 | 18 |
| reset_pb | output | BOOL | %QX2.3 | 19 |
| at_home | output | BOOL | %QX2.4 | 20 |
| cycle_done | output | BOOL | %QX2.5 | 21 |
| r_fault | output | BOOL | %QX2.6 | 22 |
| ph_pick | output | BOOL | %QX2.7 | 23 |
| ph_place | output | BOOL | %QX3.0 | 24 |
| arm_busy | output | BOOL | %QX3.1 | 25 |

Internal (no location): `interrupted` local BOOL, `can_start` local BOOL.

## Rungs (top to bottom, this order)

```
R1   estop_ok(/)                                   -> ( ) estop_active        NC E-stop loop: open wire = stop
R2   estop_active                                  -> (S) estop_latch
R3   reset_pb  estop_active(/)                     -> ( ) reset_cmd
R4   reset_cmd                                     -> (R) estop_latch
R5   run  at_home(/)                               -> (S) interrupted         left home while running
R6   at_home                                       -> (R) interrupted
R7   at_home  ||  interrupted   (parallel)         -> ( ) can_start
R8   [ start_pb  can_start ]  ||  [ run ]   (parallel branches, seal-in)
       then in series: stop_pb(/) estop_latch(/) estop_active(/) r_fault(/) cycle_done(/)
                                                   -> ( ) run
R9   run(/) estop_active(/) estop_latch(/) r_fault(/) cycle_done(/)  -> ( ) st_idle
R10  run  ph_pick(/)  ph_place(/)                  -> ( ) st_run
R11  run  ph_pick                                  -> ( ) st_pick
R12  run  ph_place                                 -> ( ) st_place
R13  cycle_done                                    -> ( ) st_done
R14  r_fault                                       -> ( ) st_fault
```
