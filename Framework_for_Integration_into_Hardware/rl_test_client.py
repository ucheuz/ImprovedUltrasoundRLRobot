#!/usr/bin/env python3
"""
rl_test_client.py
test client for the rl tcp integration.
connects to the rlinput server in the qt app and sends test joint commands.
robot v3.4, 22 joints, newline delimited json over tcp.

usage:
    python3 rl_test_client.py
    python3 rl_test_client.py --test neutral
    python3 rl_test_client.py --test feedback
"""

import socket
import json
import time
import sys
import math
import argparse


# robot joint config from robotInfo.xml v3.4

NUM_JOINTS = 22

# labels matching joint order in robotInfo.xml
JOINT_LABELS = [
    "LG",    # 0  left gantry, mm
    "LP",    # 1  left passive
    "LA1a",  # 2  left arm link 1a
    "LA1b",  # 3  left arm link 1b, dependent on joint 2
    "LA2",   # 4  left arm link 2
    "LA3",   # 5  left arm link 3
    "LS1",   # 6  left surface alignment 1
    "LS2",   # 7  left surface alignment 2
    "LO1",   # 8  left orientation 1
    "LO2",   # 9  left orientation 2
    "LO3",   # 10 left orientation 3
    "RG",    # 11 right gantry, mirrors joint 0
    "RP",    # 12 right passive
    "RA1a",  # 13 right arm link 1a
    "RA1b",  # 14 right arm link 1b, dependent on joint 13
    "RA2",   # 15 right arm link 2
    "RA3",   # 16 right arm link 3
    "RS1",   # 17 right surface alignment 1
    "RS2",   # 18 right surface alignment 2
    "RO1",   # 19 right orientation 1
    "RO2",   # 20 right orientation 2
    "RO3",   # 21 right orientation 3
]

# neutral positions in degrees, mm for gantry. from jointNeutralPosition in robotInfo.xml
JOINT_NEUTRAL = [
    0.0,    # LG
    90.0,   # LP
    13.33,  # LA1a
    -13.33, # LA1b, mirrors LA1a
    15.0,   # LA2
    -65.0,  # LA3
    -28.0,  # LS1
    90.0,   # LS2
    0.0,    # LO1
    -90.0,  # LO2
    -17.0,  # LO3
    0.0,    # RG, same physical joint as LG
    90.0,   # RP
    13.33,  # RA1a
    -13.33, # RA1b, mirrors RA1a
    -15.0,  # RA2
    65.0,   # RA3
    28.0,   # RS1
    90.0,   # RS2
    0.0,    # RO1
    -90.0,  # RO2
    17.0,   # RO3
]

# joint limits from robotInfo.xml, commands outside these get rejected
JOINT_MIN = [
    -500.0,  45.0, -18.0, -42.4, -92.0, -160.0, -170.0, 84.0,
    -90.0, -138.0, -190.0,
    -500.0,  45.0, -18.0, -42.6, -105.0, -104.0, -172.5, 90.0,
    -90.0, -139.0, -180.0,
]

# max limits from robotInfo.xml
JOINT_MAX = [
    500.0, 180.0, 42.4,  18.0,  68.0, 135.0, 173.0, 205.0,
    86.0,  -40.0, 170.0,
    500.0, 180.0, 42.6,  18.0,  58.0, 158.0, 170.0, 205.0,
    91.0,  -40.0, 180.0,
]

# dependent joints: 3 mirrors 2, 14 mirrors 13
DEPENDENT_JOINTS = {3: 2, 14: 13}

# passive unmotorised joints
PASSIVE_JOINTS = [1, 12]


# tcp connection defaults

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 12345
SOCKET_TIMEOUT = 5.0
BUFFER_SIZE = 4096


# helper functions

def clamp_joint_angle(index, value):
    """clamp a joint value to its min max range."""
    clamped = max(JOINT_MIN[index], min(JOINT_MAX[index], value))
    if clamped != value:
        print(f"  [WARN] Joint {index} ({JOINT_LABELS[index]}): "
              f"{value:.2f} clamped to {clamped:.2f} "
              f"(range [{JOINT_MIN[index]:.1f}, {JOINT_MAX[index]:.1f}])")
    return clamped


def enforce_dependent_joints(angles):
    """override dependent joints 3 and 14 to be the negative of their parent."""
    for dep_idx, parent_idx in DEPENDENT_JOINTS.items():
        expected = -angles[parent_idx]
        if abs(angles[dep_idx] - expected) > 0.01:
            print(f"  [INFO] Joint {dep_idx} ({JOINT_LABELS[dep_idx]}) "
                  f"overridden: {angles[dep_idx]:.2f} → {expected:.2f} "
                  f"(depends on joint {parent_idx})")
            angles[dep_idx] = expected


def build_joint_message(angles):
    """pack joint angles into newline delimited json bytes for sending."""
    assert len(angles) == NUM_JOINTS, \
        f"Expected {NUM_JOINTS} joint angles, got {len(angles)}"

    message = json.dumps({"joint_angles": angles})
    return (message + "\n").encode("utf-8")


def print_joint_table(angles, label="Joint Angles"):
    """print a table of joints showing angle, neutral, and offset."""
    print(f"\n{'=' * 60}")
    print(f"  {label}")
    print(f"{'=' * 60}")
    print(f"  {'Idx':<5} {'Label':<7} {'Angle':>10} {'Neutral':>10} {'Offset':>10}")
    print(f"  {'-' * 5} {'-' * 7} {'-' * 10} {'-' * 10} {'-' * 10}")
    for i in range(NUM_JOINTS):
        offset = angles[i] - JOINT_NEUTRAL[i]
        marker = ""
        if i in PASSIVE_JOINTS:
            marker = " (passive)"
        elif i in DEPENDENT_JOINTS:
            marker = " (dependent)"
        print(f"  {i:<5} {JOINT_LABELS[i]:<7} {angles[i]:>10.2f} "
              f"{JOINT_NEUTRAL[i]:>10.2f} {offset:>+10.2f}{marker}")
    print(f"{'=' * 60}\n")


# test scenarios

def test_neutral():
    """send neutral pose, robot shouldn't move."""
    return list(JOINT_NEUTRAL)


def test_small_perturbation():
    """small offsets on a few safe joints to check signal flow."""
    angles = list(JOINT_NEUTRAL)
    angles[4] += 5.0    # LA2: nudge left elbow
    angles[15] -= 5.0   # RA2: nudge right elbow
    angles[6] += 3.0    # LS1: tilt left surface alignment
    return angles


def test_left_arm_only():
    """move only the left arm, right stays at neutral."""
    angles = list(JOINT_NEUTRAL)
    angles[0] = 50.0    # LG: gantry 50mm left
    angles[4] = 40.0    # LA2: elbow extended
    angles[5] = -100.0  # LA3: shoulder rotated
    angles[6] = -50.0   # LS1: surface alignment tilted
    return angles


def test_right_arm_only():
    """move only the right arm. note joint 11 is shared with joint 0."""
    angles = list(JOINT_NEUTRAL)
    angles[11] = -50.0   # RG: gantry 50mm right
    angles[15] = -40.0   # RA2: elbow extended
    angles[16] = 100.0   # RA3: shoulder rotated
    angles[17] = 50.0    # RS1: surface alignment tilted
    return angles


def test_sinusoidal_sweep(t):
    """generate sine wave joint angles for time t, simulates continuous rl commands."""
    amplitude = 5.0
    frequency = 0.2
    angles = list(JOINT_NEUTRAL)

    for i in range(NUM_JOINTS):
        # skip passive and dependent
        if i in PASSIVE_JOINTS or i in DEPENDENT_JOINTS:
            continue

        # phase offset per joint
        phase = i * (2.0 * math.pi / NUM_JOINTS)
        offset = amplitude * math.sin(2.0 * math.pi * frequency * t + phase)
        angles[i] = JOINT_NEUTRAL[i] + offset

    enforce_dependent_joints(angles)
    return angles


def test_boundary():
    """push joints to limits and slightly beyond to test clamping. sim mode only."""
    angles = list(JOINT_NEUTRAL)
    # left arm extremes
    angles[4] = JOINT_MAX[4]       # LA2 at max
    angles[5] = JOINT_MIN[5]       # LA3 at min
    angles[6] = JOINT_MAX[6] + 10  # LS1 intentionally over max, should clamp
    # right arm extremes
    angles[15] = JOINT_MIN[15]     # RA2 at min
    angles[16] = JOINT_MAX[16]     # RA3 at max
    angles[17] = JOINT_MIN[17] - 5 # RS1 intentionally under min, should clamp
    return angles


# tcp connection and send receive

def connect_to_robot(host, port):
    """connect to the rlinput tcp server. exits if connection fails."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(SOCKET_TIMEOUT)

    print(f"[CONNECTING] Attempting TCP connection to {host}:{port}...")
    try:
        sock.connect((host, port))
        print(f"[CONNECTED]  Successfully connected to RLInput server.")
        return sock
    except ConnectionRefusedError:
        print(f"[ERROR] Connection refused at {host}:{port}.")
        print(f"        Checklist:")
        print(f"          1. Is the robot control app running?")
        print(f"          2. Did you click 'Connect External Inputs'?")
        print(f"          3. Is the port set to {port} in the RL Input tab?")
        sys.exit(1)
    except socket.timeout:
        print(f"[ERROR] Connection timed out after {SOCKET_TIMEOUT}s.")
        sys.exit(1)


def send_joint_angles(sock, angles):
    """clamp, enforce dependent joints, serialise, and send over tcp."""
    clamped = [clamp_joint_angle(i, angles[i]) for i in range(NUM_JOINTS)]
    enforce_dependent_joints(clamped)
    message = build_joint_message(clamped)

    try:
        sock.sendall(message)
        return True
    except (BrokenPipeError, ConnectionResetError) as e:
        print(f"[ERROR] Send failed: {e}")
        print(f"        The C++ application may have disconnected.")
        return False


def receive_feedback(sock):
    """try to read one feedback json message. returns parsed dict or none."""
    try:
        sock.settimeout(0.5)
        data = sock.recv(BUFFER_SIZE)
        if data:
            lines = data.decode("utf-8").strip().split("\n")
            for line in reversed(lines):
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
        return None
    except socket.timeout:
        return None
    except (ConnectionResetError, BrokenPipeError):
        print("[WARN] Connection lost while waiting for feedback.")
        return None
    finally:
        sock.settimeout(SOCKET_TIMEOUT)


def receive_all_feedback(sock, max_attempts=10):
    """read all available feedback, positions and forces. returns dict with both or none."""
    result = {"joint_positions": None, "forces": None}

    for _ in range(max_attempts):
        try:
            sock.settimeout(0.3)
            data = sock.recv(BUFFER_SIZE)
            if not data:
                break
            lines = data.decode("utf-8").strip().split("\n")
            for line in lines:
                try:
                    msg = json.loads(line)
                    msg_type = msg.get("type", "")
                    if msg_type == "joint_positions":
                        result["joint_positions"] = msg.get("data", [])
                    elif msg_type == "forces":
                        result["forces"] = msg.get("data", [])
                except json.JSONDecodeError:
                    continue
            if result["joint_positions"] is not None and result["forces"] is not None:
                break
        except socket.timeout:
            break
        except (ConnectionResetError, BrokenPipeError):
            print("[WARN] Connection lost while waiting for feedback.")
            break
    sock.settimeout(SOCKET_TIMEOUT)
    return result


# test runner

def run_single_test(sock, name, angles):
    """send one test case, wait briefly, check for feedback."""
    print(f"\n{'─' * 60}")
    print(f"  TEST: {name}")
    print(f"{'─' * 60}")

    print_joint_table(angles, label=f"Commanding: {name}")

    success = send_joint_angles(sock, angles)
    if not success:
        print(f"  [FAIL] Could not send joint angles.")
        return

    print(f"  [SENT] Joint angles transmitted successfully.")

    # wait for processing
    time.sleep(0.5)

    feedback = receive_feedback(sock)
    if feedback:
        print(f"  [FEEDBACK] Received: {json.dumps(feedback, indent=2)}")
    else:
        print(f"  [FEEDBACK] No feedback received (may not be wired yet).")


def run_sweep_test(sock, duration=10.0, rate=10.0):
    """run the sine sweep for a given duration and command rate."""
    print(f"\n{'─' * 60}")
    print(f"  TEST: Sinusoidal Sweep ({duration}s at {rate} Hz)")
    print(f"{'─' * 60}")

    interval = 1.0 / rate
    steps = int(duration * rate)

    for step in range(steps):
        t = step * interval
        angles = test_sinusoidal_sweep(t)
        success = send_joint_angles(sock, angles)

        if not success:
            print(f"  [ABORT] Connection lost at step {step}/{steps}.")
            return

        if step % int(rate) == 0:
            print(f"  [SWEEP] t={t:.1f}s — "
                  f"LA2={angles[4]:.2f}° RA2={angles[15]:.2f}°")

        # maintain command rate
        time.sleep(interval)

    print(f"  [DONE] Sweep complete: {steps} commands sent over {duration}s.")


def run_feedback_test(sock):
    """send neutral and check if joint position or force feedback comes back."""
    print(f"\n{'─' * 60}")
    print(f"  TEST: Feedback Channel Verification")
    print(f"{'─' * 60}")

    angles = list(JOINT_NEUTRAL)
    success = send_joint_angles(sock, angles)
    if not success:
        print("  [FAIL] Could not send joint angles.")
        return

    print("  [SENT] Neutral pose transmitted.")

    # give the update timer time to fire
    time.sleep(1.0)

    result = receive_all_feedback(sock)

    positions = result["joint_positions"]
    forces = result["forces"]

    print()
    if positions is not None:
        print(f"  [PASS] Joint position feedback received ({len(positions)} values)")
        if len(positions) >= NUM_JOINTS:
            max_diff = max(abs(positions[i] - angles[i]) for i in range(NUM_JOINTS))
            print(f"         Max deviation from commanded: {max_diff:.2f}°")
            if max_diff < 5.0:
                print(f"         Positions match commanded pose (within 5°)")
            else:
                print(f"         Large deviation — motors may still be moving")
            print()
            print(f"  {'Joint':<7} {'Commanded':>10} {'Actual':>10} {'Diff':>10}")
            print(f"  {'-' * 7} {'-' * 10} {'-' * 10} {'-' * 10}")
            for i in range(min(NUM_JOINTS, len(positions))):
                diff = positions[i] - angles[i]
                if abs(diff) > 0.01:
                    print(f"  {JOINT_LABELS[i]:<7} {angles[i]:>10.2f} "
                          f"{positions[i]:>10.2f} {diff:>+10.2f}")
    else:
        print(f"  [FAIL] No joint position feedback received.")
        print(f"         → Check that setCurrentJointPositions() is wired in")
        print(f"           RobotControl::updateCurrentPosition()")

    print()
    if forces is not None:
        print(f"  [PASS] Force feedback received ({len(forces)} values)")
        print(f"         Values: {[round(f, 3) for f in forces]}")
    else:
        print(f"  [INFO] No force feedback received.")
        print(f"         This is normal if force sensors are not connected.")

    print(f"\n  {'─' * 56}")
    if positions is not None:
        print(f"  FEEDBACK TEST: PASSED — closed-loop RL is ready")
    else:
        print(f"  FEEDBACK TEST: NOT YET WIRED — see instructions above")
    print(f"  {'─' * 56}")


def main():
    """parse args, connect, and run selected tests."""
    parser = argparse.ArgumentParser(
        description="Test client for RLInput TCP server (Robot V3.4)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 rl_test_client.py                     # Run all tests
  python3 rl_test_client.py --test neutral      # Just send neutral pose
  python3 rl_test_client.py --test feedback     # Test feedback channel only
  python3 rl_test_client.py --test sweep --sweep-duration 30
  python3 rl_test_client.py --host 192.168.1.50 # Connect to remote machine
        """
    )
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help=f"RLInput server IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"RLInput server port (default: {DEFAULT_PORT})")
    parser.add_argument("--test", default="all",
                        choices=["all", "neutral", "perturb", "left",
                                 "right", "sweep", "boundary", "feedback"],
                        help="Which test scenario to run (default: all)")
    parser.add_argument("--sweep-duration", type=float, default=10.0,
                        help="Duration of sweep test in seconds (default: 10)")
    parser.add_argument("--sweep-rate", type=float, default=10.0,
                        help="Command rate for sweep test in Hz (default: 10)")
    args = parser.parse_args()

    print("=" * 60)
    print("  RL Test Client for Robot V3.4")
    print("  KCL Ultrasound Robot Control — RL Integration")
    print(f"  Target: {args.host}:{args.port}")
    print(f"  Test:   {args.test}")
    print("=" * 60)

    sock = connect_to_robot(args.host, args.port)

    try:
        if args.test in ("all", "neutral"):
            run_single_test(sock, "Neutral Pose", test_neutral())
            time.sleep(1.0)

        if args.test in ("all", "perturb"):
            run_single_test(sock, "Small Perturbation", test_small_perturbation())
            time.sleep(1.0)

        if args.test in ("all", "left"):
            run_single_test(sock, "Left Arm Only", test_left_arm_only())
            time.sleep(1.0)

        if args.test in ("all", "right"):
            run_single_test(sock, "Right Arm Only", test_right_arm_only())
            time.sleep(1.0)

        if args.test in ("all", "boundary"):
            run_single_test(sock, "Boundary Test", test_boundary())
            time.sleep(1.0)

        if args.test in ("all", "sweep"):
            run_sweep_test(sock, args.sweep_duration, args.sweep_rate)

        if args.test in ("all", "feedback"):
            run_feedback_test(sock)
            time.sleep(1.0)

        if args.test == "all":
            print("\n[RESET] Returning to neutral pose...")
            send_joint_angles(sock, test_neutral())
            time.sleep(1.0)

        print("\n[COMPLETE] All tests finished successfully.")

    except KeyboardInterrupt:
        # ctrl c, go back to neutral before quitting
        print("\n\n[INTERRUPTED] Ctrl+C detected. Returning to neutral...")
        try:
            send_joint_angles(sock, test_neutral())
            time.sleep(0.5)
        except Exception:
            pass
        print("[EXIT] Disconnected.")

    finally:
        # close socket
        sock.close()
        print("[SOCKET] Connection closed.")


if __name__ == "__main__":
    main()
