PROGRAM main
  VAR
    interrupted : BOOL;
    can_start : BOOL;
  END_VAR
  VAR
    run AT %QX0.1 : BOOL;
    estop_active AT %QX0.2 : BOOL;
    reset_cmd AT %QX0.3 : BOOL;
    st_idle AT %QX0.4 : BOOL;
    st_run AT %QX0.5 : BOOL;
    st_pick AT %QX0.6 : BOOL;
    st_place AT %QX0.7 : BOOL;
    st_done AT %QX1.0 : BOOL;
    estop_latch AT %QX1.1 : BOOL;
    st_fault AT %QX1.2 : BOOL;
    start_pb AT %QX2.0 : BOOL;
    stop_pb AT %QX2.1 : BOOL;
    estop_ok AT %QX2.2 : BOOL;
    reset_pb AT %QX2.3 : BOOL;
    at_home AT %QX2.4 : BOOL;
    cycle_done AT %QX2.5 : BOOL;
    r_fault AT %QX2.6 : BOOL;
    ph_pick AT %QX2.7 : BOOL;
    ph_place AT %QX3.0 : BOOL;
    arm_busy AT %QX3.1 : BOOL;
  END_VAR

  estop_active := NOT(estop_ok);
  IF estop_active THEN
    estop_latch := TRUE; (*set*)
  END_IF;
  reset_cmd := NOT(estop_active) AND reset_pb;
  IF reset_cmd THEN
    estop_latch := FALSE; (*reset*)
  END_IF;
  IF NOT(at_home) AND run THEN
    interrupted := TRUE; (*set*)
  END_IF;
  IF at_home THEN
    interrupted := FALSE; (*reset*)
  END_IF;
  can_start := at_home OR interrupted;
  run := NOT(cycle_done) AND NOT(r_fault) AND NOT(estop_active) AND NOT(estop_latch) AND NOT(stop_pb) AND (run OR can_start AND start_pb);
  st_idle := NOT(cycle_done) AND NOT(r_fault) AND NOT(estop_latch) AND NOT(estop_active) AND NOT(run);
  st_run := NOT(ph_place) AND NOT(ph_pick) AND run;
  st_pick := ph_pick AND run;
  st_place := ph_place AND run;
  st_done := cycle_done;
  st_fault := r_fault;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 1);
    PROGRAM instance0 WITH task0 : main;
  END_RESOURCE
END_CONFIGURATION
