import time
import msvcrt
from python_st3215 import ST3215, ServoNotRespondingError

LEADER_PORT = "COM5"
FOLLOWER_PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_ID = 6
RATE_HZ = 40

LEADER_LIMITS = {
    1: (752, 3443),
    2: (804, 3209),
    3: (770, 3097),
    4: (843, 2911),
    5: (0, 4095),
    6: (700, 2500),   # includes leader rest 2301
}
FOLLOWER_LIMITS = {
    1: (0, 4095),
    2: (0, 4095),
    3: (0, 4095),
    4: (0, 4095),
    5: (0, 4095),
    6: (1200, 2900),  # real travel, not a dead 1650–2450 sliver
}

LEADER_REST = {
    1: 2048,
    2: 759,
    3: 3054,
    4: 1926,
    5: 3385,
    6: 2301,
}
FOLLOWER_REST = {
    1: 2048,
    2: 2048,
    3: 2048,
    4: 2048,
    5: 2048,
    6: 2050,
}

INVERT = {1: False, 2: False, 3: False, 4: False, 5: False, 6: False}

BODY_SPEED = 2500
BODY_ACCEL = 50
GRIPPER_SPEED = 700
GRIPPER_ACCEL = 20
FOLLOW_SMOOTH = 0.75
DEADBAND = 3
REST_TIMEOUT_S = 8.0
PRINT_EVERY_S = 1.0


def torque_on(servo):
    try:
        servo.sram.write_torque_switch(1)
        return
    except Exception:
        pass
    try:
        servo.sram.torque_enable()
    except Exception:
        pass


def torque_off(servo):
    try:
        servo.sram.write_torque_switch(0)
        return
    except Exception:
        pass
    try:
        servo.sram.torque_disable()
    except Exception:
        pass


def set_profile(servo, speed, accel):
    try:
        if hasattr(servo.sram, "write_acceleration"):
            servo.sram.write_acceleration(accel)
        if hasattr(servo.sram, "write_running_speed"):
            servo.sram.write_running_speed(speed)
        elif hasattr(servo.sram, "write_goal_speed"):
            servo.sram.write_goal_speed(speed)
    except Exception:
        pass
    try:
        if hasattr(servo.sram, "write_register"):
            servo.sram.write_register(41, accel, 1)
            servo.sram.write_register(46, speed, 2)
    except Exception:
        pass


def is_moving(servo):
    for name in ("read_moving", "read_mobile_sign"):
        try:
            fn = getattr(servo.sram, name, None)
            if callable(fn):
                return bool(fn())
        except Exception:
            pass
    return False


def connect(port, ids):
    bus = ST3215(port)
    servos = {}
    for sid in ids:
        try:
            servos[sid] = bus.wrap_servo(sid)
            print(f"{port}  ID {sid}: ok")
        except ServoNotRespondingError:
            print(f"{port}  ID {sid}: NO RESPONSE")
    return bus, servos


def read_pose(bus, servos):
    ids = list(servos.keys())
    if hasattr(bus, "broadcast"):
        try:
            raw = bus.broadcast.sram.sync_read_current_location(ids)
            if isinstance(raw, dict) and raw:
                return {int(k): int(v) for k, v in raw.items() if v is not None}
        except Exception:
            pass
    pose = {}
    for sid, servo in servos.items():
        try:
            pose[sid] = int(servo.sram.read_current_location())
        except Exception:
            pass
    return pose


def dispatch(bus, servos, targets):
    live = {sid: pos for sid, pos in targets.items() if sid in servos}
    if not live:
        return
    if hasattr(bus, "sync_write_target_location"):
        try:
            bus.sync_write_target_location(live)
            return
        except Exception:
            pass
    if hasattr(bus, "broadcast"):
        try:
            bus.broadcast.sram.sync_write_target_location(live)
            return
        except Exception:
            pass
    for sid, pos in live.items():
        try:
            servos[sid].sram.write_target_location(pos)
        except Exception:
            pass


def wait_still(servos, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if not any(is_moving(s) for s in servos.values()):
            time.sleep(0.2)
            return
        time.sleep(0.05)


def map_joint(pos, sid):
    smin, smax = LEADER_LIMITS[sid]
    dmin, dmax = FOLLOWER_LIMITS[sid]
    if smax == smin:
        return FOLLOWER_REST[sid]
    pos = max(smin, min(smax, pos))
    u = (pos - smin) / float(smax - smin)
    if INVERT.get(sid, False):
        u = 1.0 - u
    out = int(round(dmin + u * (dmax - dmin)))
    return max(dmin, min(dmax, out))


def map_pose(pose):
    return {
        sid: map_joint(pos, sid)
        for sid, pos in pose.items()
        if sid in LEADER_LIMITS and sid in FOLLOWER_LIMITS
    }


def blend(prev, nxt, alpha):
    if prev is None:
        return dict(nxt)
    return {
        sid: int(round(prev.get(sid, pos) + alpha * (pos - prev.get(sid, pos))))
        for sid, pos in nxt.items()
    }


def changed_enough(prev, nxt, band):
    if prev is None:
        return True
    return any(abs(pos - prev.get(sid, pos)) >= band for sid, pos in nxt.items())


def go_rest(bus, servos, rest, label):
    print(f"{label}: moving to rest...")
    for sid, servo in servos.items():
        torque_on(servo)
        if sid == GRIPPER_ID:
            set_profile(servo, GRIPPER_SPEED, GRIPPER_ACCEL)
        else:
            set_profile(servo, 800, 25)
    dispatch(bus, servos, {sid: rest[sid] for sid in servos if sid in rest})
    wait_still(servos, REST_TIMEOUT_S)
    print(f"{label} rest now:", read_pose(bus, servos))


def limp_all(leader, follower):
    for servo in list(leader.values()) + list(follower.values()):
        torque_off(servo)
    print("Torque OFF on both arms (t). Press f to follow again.")


def arm_follower(follower):
    for sid, servo in follower.items():
        torque_on(servo)
        if sid == GRIPPER_ID:
            set_profile(servo, GRIPPER_SPEED, GRIPPER_ACCEL)
        else:
            set_profile(servo, BODY_SPEED, BODY_ACCEL)


def poll_key():
    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0") and msvcrt.kbhit():
        msvcrt.getch()
        return None
    try:
        return ch.decode("ascii").lower()
    except Exception:
        return None


print(f"Leader   {LEADER_PORT}")
print(f"Follower {FOLLOWER_PORT}")
leader_bus = follower_bus = None
try:
    leader_bus, leader = connect(LEADER_PORT, SERVO_IDS)
    follower_bus, follower = connect(FOLLOWER_PORT, SERVO_IDS)
    if not leader or not follower:
        raise RuntimeError("Need servos on both ports.")

    go_rest(leader_bus, leader, LEADER_REST, "Leader")
    go_rest(follower_bus, follower, FOLLOWER_REST, "Follower")

    print("Mapped at rest. Leader limp.")
    for servo in leader.values():
        torque_off(servo)
    arm_follower(follower)

    print("Following. Keys: t = limp both, f = follow again, q = quit")
    dt = 1.0 / RATE_HZ
    filtered = None
    last_sent = None
    last_print = 0.0
    n = 0
    following = True

    while True:
        t0 = time.perf_counter()
        key = poll_key()
        if key == "q":
            break
        if key == "t":
            following = False
            limp_all(leader, follower)
        elif key == "f":
            following = True
            for servo in leader.values():
                torque_off(servo)
            arm_follower(follower)
            filtered = None
            last_sent = None
            print("Following again.")

        n += 1
        if following and n % 50 == 1:
            for servo in leader.values():
                torque_off(servo)

        if following:
            raw = map_pose(read_pose(leader_bus, leader))
            if raw:
                filtered = blend(filtered, raw, FOLLOW_SMOOTH)
                if changed_enough(last_sent, filtered, DEADBAND):
                    dispatch(follower_bus, follower, filtered)
                    last_sent = filtered
                now = time.perf_counter()
                if now - last_print >= PRINT_EVERY_S:
                    print(
                        f"lead6={read_pose(leader_bus, leader).get(6)}  "
                        f"follow6={filtered.get(6)}"
                    )
                    last_print = now

        sleep = dt - (time.perf_counter() - t0)
        if sleep > 0:
            time.sleep(sleep)
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    limp_all(leader, follower) if leader_bus else None
    if leader_bus:
        leader_bus.close()
    if follower_bus:
        follower_bus.close()
    print("Ports closed.")