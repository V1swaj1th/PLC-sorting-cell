#!/usr/bin/env python3
"""Checks the ladder program running in OpenPLC through Modbus TCP, with this script playing the
ROS side. Mirrors the software-PLC test (plc_link_test). Run it with NO ROS bridge running.

    python3 plc_check.py [host] [port]
"""
import sys
import time

from pymodbus.client import ModbusTcpClient

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 5020

IN = ["start_pb", "stop_pb", "estop_ok", "reset_pb", "at_home", "cycle_done", "fault",
      "ph_pick", "ph_place", "arm_busy", "drive_fault"]       # coils 16..26 (ROS -> PLC)
OUT = ["lamp", "run", "estop_active", "reset_cmd", "st_idle", "st_run", "st_pick",
       "st_place", "st_done", "estop_latch", "st_fault", "drive_alarm"]  # coils 0..11 (PLC -> ROS)

c = ModbusTcpClient(HOST, port=PORT)
if not c.connect():
    sys.exit("cannot connect to %s:%d" % (HOST, PORT))

state = {k: False for k in IN}
passed = failed = 0


def push():
    r = c.write_coils(16, [state[k] for k in IN])
    if r.isError():
        sys.exit("write_coils failed: %s" % r)


def read():
    r = c.read_coils(0, count=len(OUT))
    if r.isError():
        sys.exit("read_coils failed: %s" % r)
    return dict(zip(OUT, r.bits[:len(OUT)]))


def step(t=0.3):
    push()
    time.sleep(t)
    return read()


def pulse(name):
    state[name] = True
    step()
    state[name] = False
    return step()


def name_of(o):
    if o["estop_active"] or o["estop_latch"]:
        return "E-STOP"
    for k, n in (("drive_alarm", "DRIVE_ALARM"), ("st_fault", "FAULT"), ("st_done", "DONE"), ("st_pick", "PICK"),
                 ("st_place", "PLACE"), ("st_run", "RUN"), ("st_idle", "IDLE")):
        if o[k]:
            return n
    return "UNKNOWN"


def expect(what, cond):
    global passed, failed
    print("  [%s] %s" % ("PASS" if cond else "FAIL", what))
    passed += cond
    failed += (not cond)


print("0. Coil write persistence (master-written coils 16..26 must survive PLC scans)")
state.update(estop_ok=True, at_home=True)
step(0.5)
r = c.read_coils(16, count=len(IN))
back = dict(zip(IN, r.bits[:len(IN)]))
expect("coils 16..25 read back as written", back["estop_ok"] and back["at_home"] and not back["start_pb"])

print("1. E-stop loop open -> E-STOP; closing the loop is not enough, reset is needed")
state["estop_ok"] = False
o = step()
expect("estop_active", o["estop_active"])
state["estop_ok"] = True
o = step()
expect("latch still set", o["estop_latch"] and name_of(o) == "E-STOP")
o = pulse("start_pb")
expect("start refused while latched", not o["run"])

print("2. Reset")
o = pulse("reset_pb")
expect("latch cleared, state IDLE", not o["estop_latch"] and name_of(o) == "IDLE")

print("3. Start at home, phases")
o = pulse("start_pb")
expect("run held after release, state RUN", o["run"] and name_of(o) == "RUN")
state.update(at_home=False, arm_busy=True, ph_pick=True)
expect("PICK", name_of(step()) == "PICK")
state.update(ph_pick=False, ph_place=True)
expect("PLACE", name_of(step()) == "PLACE")

print("4. E-stop mid-motion, reset, resume")
state["estop_ok"] = False
o = step()
expect("run dropped, E-STOP", (not o["run"]) and name_of(o) == "E-STOP")
state.update(estop_ok=True, arm_busy=False)
step()
o = pulse("start_pb")
expect("start refused before reset", not o["run"])
o = pulse("reset_pb")
expect("latch cleared", not o["estop_latch"])
o = pulse("start_pb")
expect("resume accepted away from home", o["run"])

print("5. Stop button, resume")
o = pulse("stop_pb")
expect("run dropped", not o["run"])
o = pulse("start_pb")
expect("resume allowed", o["run"])

print("6. Cycle done")
state.update(ph_place=False, at_home=True, cycle_done=True)
o = step()
expect("run dropped, state DONE", (not o["run"]) and name_of(o) == "DONE")
o = pulse("start_pb")
expect("start refused while cycle_done", not o["run"])
state["cycle_done"] = False
expect("IDLE again", name_of(step()) == "IDLE")

print("7. Fault interlock")
state["fault"] = True
o = step()
expect("FAULT", name_of(o) == "FAULT")
o = pulse("start_pb")
expect("start refused with a fault", not o["run"])
state["fault"] = False
step()

print("8. Start interlock: arm not at home, nothing interrupted")
state["at_home"] = False
step()
o = pulse("start_pb")
expect("start refused", not o["run"])
state["at_home"] = True
step()
o = pulse("start_pb")
expect("start accepted at home", o["run"])

print("9. Servo drive fault: PLC latches drive_alarm")
state["at_home"] = False
o = step()
expect("running before the fault", o["run"])
state["drive_fault"] = True
o = step()
expect("drive_alarm latched, run dropped, state DRIVE_ALARM",
       o["drive_alarm"] and not o["run"] and name_of(o) == "DRIVE_ALARM")
o = pulse("reset_pb")
expect("reset refused while the drive still reports the fault", o["drive_alarm"])
state["drive_fault"] = False
o = step()
expect("alarm stays latched after the drive fault clears", o["drive_alarm"])
o = pulse("start_pb")
expect("start refused while the alarm is latched", not o["run"])
o = pulse("reset_pb")
expect("reset clears the alarm", not o["drive_alarm"])
o = pulse("start_pb")
expect("start accepted again (resume)", o["run"])

# leave the PLC in a safe state
state.update(estop_ok=False)
step()
c.close()
print("\n%d passed, %d failed" % (passed, failed))
sys.exit(0 if failed == 0 else 1)
