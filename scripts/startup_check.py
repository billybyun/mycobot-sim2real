#!/usr/bin/env python3
import time
from pymycobot import MyCobot, PI_PORT, PI_BAUD

PARK = [0, 0, 0, 0, 0, 0]
TEST_POSE = [0, -60, 90, 0, 45, 0]

SPEED = 15         # 1-100 (lower = slower)
TOL_DEG = 10       # pass/fail tolerance for reaching target
WAIT_S = 10        # how long to wait for motion


def deg_abs_diff(a, b):
    d = a - b
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return abs(d)


def wait_reached(mc, target, timeout_s=10, tol_deg=10, poll_s=0.25):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout_s:
        ang = mc.get_angles()
        # get_angles() can return int (e.g. -1) on comm errors; guard before len()
        if isinstance(ang, (list, tuple)) and len(ang) >= 6:
            last = ang[:6]
            errs = [deg_abs_diff(last[i], target[i]) for i in range(6)]
            if max(errs) <= tol_deg:
                return True, last, errs
        time.sleep(poll_s)
    if last is None:
        return False, None, None
    errs = [deg_abs_diff(last[i], target[i]) for i in range(6)]
    return False, last, errs


def main():
    print(f"[INFO] Using PI_PORT={PI_PORT}  PI_BAUD={PI_BAUD}")
    mc = MyCobot(PI_PORT, PI_BAUD)

    # Warm up comms
    for _ in range(3):
        a = mc.get_angles()
        if isinstance(a, (list, tuple)) and len(a) >= 6:
            break
        time.sleep(0.2)

    print("[INFO] power_on()")
    mc.power_on()
    time.sleep(0.8)

    ang0 = mc.get_angles()
    if not isinstance(ang0, (list, tuple)) or len(ang0) < 6:
        print("[FAIL] Could not read angles. Check wiring/power/port.")
        return 2
    print("[INFO] Current angles:", [round(x, 2) for x in ang0[:6]])

    print("\n[SAFETY] Clear workspace. Moving to PARK then TEST pose.\n")

    print("[INFO] Move -> PARK:", PARK)
    ret = mc.send_angles(PARK, SPEED)
    if ret == -1:
        print("[WARN] send_angles returned -1 (no ACK). Will verify by get_angles().")
    ok, reached, errs = wait_reached(mc, PARK, timeout_s=WAIT_S, tol_deg=TOL_DEG)
    print("[OK]" if ok else "[WARN]", "PARK reached:", ok,
          "reached:", None if reached is None else [round(x, 2) for x in reached],
          "max_err:", None if errs is None else round(max(errs), 2))

    print("\n[INFO] Move -> TEST_POSE:", TEST_POSE)
    ret = mc.send_angles(TEST_POSE, SPEED)
    if ret == -1:
        print("[WARN] send_angles returned -1 (no ACK). Will verify by get_angles().")
    ok, reached, errs = wait_reached(mc, TEST_POSE, timeout_s=WAIT_S, tol_deg=TOL_DEG)
    print("[OK]" if ok else "[WARN]", "TEST_POSE reached:", ok,
          "reached:", None if reached is None else [round(x, 2) for x in reached],
          "max_err:", None if errs is None else round(max(errs), 2))

    print("\n[INFO] Return -> PARK:", PARK)
    ret = mc.send_angles(PARK, SPEED)
    if ret == -1:
        print("[WARN] send_angles returned -1 (no ACK). Will verify by get_angles().")
    ok, reached, errs = wait_reached(mc, PARK, timeout_s=WAIT_S, tol_deg=TOL_DEG)
    print("[OK]" if ok else "[WARN]", "Final PARK reached:", ok,
          "reached:", None if reached is None else [round(x, 2) for x in reached],
          "max_err:", None if errs is None else round(max(errs), 2))

    # Gripper test (adjust if your open/close are reversed)
    print("\n[INFO] Gripper test (you may need to swap open/close values)")
    try:
        OPEN_VAL = 0
        CLOSE_VAL = 100
        mc.set_gripper_value(OPEN_VAL, 50)
        time.sleep(1.0)
        mc.set_gripper_value(CLOSE_VAL, 50)
        time.sleep(1.0)
        mc.set_gripper_value(OPEN_VAL, 50)
        time.sleep(0.8)
        print("[OK] Gripper commands sent.")
    except Exception as e:
        print("[WARN] Gripper command failed (maybe no gripper / API mismatch):", e)

    print("\n[DONE] Basic PI_PORT verification complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
