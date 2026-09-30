"""Start the SO-101 stack: connect both arms, serve the loopback API, run the loop."""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from serial.tools import list_ports

from fm_teleop_so101.stack import Refused, Stack

log = logging.getLogger("fm_teleop_so101")

#: The largest request body read. Every valid body is a few dozen bytes.
MAX_BODY = 4096


def handler_for(stack: Stack):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if not self._trusted():
                return
            if self.path == "/status":
                return self._reply(200, stack.status())
            self._reply(404, {"error": f"no route {self.path}"})

        def do_POST(self):
            if not self._trusted():
                return
            if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                return self._reply(415, {"error": "body must be application/json"})
            body = self._body()
            if body is None:
                return
            try:
                if self.path == "/mode":
                    stack.set_mode(str(body.get("mode", "")))
                elif self.path == "/record" and body.get("action") == "start":
                    stack.start_take(str(body.get("dataset", "")), str(body.get("task", "")))
                elif self.path == "/record" and body.get("action") == "stop":
                    stack.stop_take()
                elif self.path == "/stop":
                    stack.stop()
                else:
                    return self._reply(404, {"error": f"no route {self.path}"})
            except Refused as refusal:
                return self._reply(409, {"ok": False, "error": str(refusal)})
            self._reply(200, {"ok": True, **stack.status()})

        def _trusted(self) -> bool:
            """Refuse anything a web browser could send on a visited page's behalf.

            Binding 127.0.0.1 keeps other hosts out, not other origins: a page in
            a browser on this machine can still reach the port. A browser always
            sends Origin on a cross-origin request and the agent never does, and
            a Host that is not loopback is a DNS-rebound name. JSON-only POSTs
            force a preflight this server never approves.
            """
            host = (self.headers.get("Host") or "").rsplit(":", 1)[0]
            if self.headers.get("Origin") is not None or host not in ("127.0.0.1", "localhost"):
                self._reply(403, {"error": "the stack answers the local agent only"})
                return False
            return True

        def _body(self) -> dict | None:
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = -1
            if not 0 <= length <= MAX_BODY:
                self._reply(400, {"error": f"body must be 0..{MAX_BODY} bytes"})
                return None
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                body = None
            if not isinstance(body, dict):
                self._reply(400, {"error": "body must be a JSON object"})
                return None
            return body

        def _reply(self, code: int, payload: dict) -> None:
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *args):
            log.debug(fmt, *args)

    return Handler


#: The one SO-101 joint that turns fully, so its 0..4095 range is not a sign
#: of a factory-fresh motor.
FULL_TURN = "wrist_roll"


def adopt_calibration(arm) -> None:
    """Take an arm's calibration from its own motors when this host has no file.

    `lerobot-calibrate` writes the homing offset and range into each motor's
    EEPROM, and the file is a copy of those values. Reading them back is what
    lets the arms move between hosts without being recalibrated. A motor still
    at factory values (offset 0, full range) was never calibrated, and adopting
    it would teach the follower the wrong zero, so that refuses instead.
    """
    arm.bus.connect()
    try:
        calibration = arm.bus.read_calibration()
    finally:
        arm.bus.disconnect(disable_torque=False)
    fresh = [
        motor
        for motor, cal in calibration.items()
        if motor != FULL_TURN and cal.homing_offset == 0 and (cal.range_min, cal.range_max) == (0, 4095)
    ]
    if fresh:
        sys.exit(f"{arm} motors {fresh} hold factory values; run lerobot-calibrate --*.id={arm.id} first")
    arm.calibration = calibration
    arm.bus.calibration = calibration
    arm._save_calibration()
    log.info("adopted %s calibration from its motors into %s", arm, arm.calibration_fpath)


def serial_port(spec: str) -> str:
    """A port path, or `serial:<n>` for the USB board whose serial number starts with n.

    The board's serial number travels with the arm; a /dev path does not — it is
    ttyACM0 on one host, tty.usbmodem5B3D… on another, and can swap at boot.
    """
    if not spec.startswith("serial:"):
        return spec
    wanted = spec.removeprefix("serial:")
    found = [p.device for p in list_ports.comports() if (p.serial_number or "").startswith(wanted)]
    # macOS lists each board twice (cu.* and tty.*); LeRobot's own find-port uses tty.
    found = sorted(set(found), key=lambda d: "/cu." in d)
    if not found:
        raise argparse.ArgumentTypeError(f"no USB serial board with serial number {wanted}; is the arm plugged in?")
    return found[0]


def camera(spec: str) -> tuple[str, OpenCVCameraConfig]:
    """`name=index_or_path[@WxH]` → an OpenCV camera at the stack's fps."""
    name, _, source = spec.partition("=")
    source, _, size = source.partition("@")
    width, _, height = size.partition("x")
    return name, OpenCVCameraConfig(
        index_or_path=int(source) if source.isdigit() else Path(source),
        width=int(width) if width else None,
        height=int(height) if height else None,
    )


def main() -> None:
    parser = argparse.ArgumentParser(prog="fm-teleop-so101", description=__doc__)
    parser.add_argument("--leader-port", required=True, type=serial_port, help="path, or serial:<USB serial>")
    parser.add_argument("--follower-port", required=True, type=serial_port, help="path, or serial:<USB serial>")
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-id", required=True)
    parser.add_argument("--owner", required=True, help="dataset owner, the robot's fleet name")
    parser.add_argument("--root", required=True, type=Path, help="directory that holds the datasets")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera", action="append", default=[], type=camera, metavar="NAME=SRC[@WxH]")
    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=10.0,
        help="largest step per control tick, in degrees; the follower ramps instead of jumping",
    )
    parser.add_argument("--port", type=int, default=8790, help="loopback API port")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    cameras = dict(args.camera)
    for cam in cameras.values():
        cam.fps = args.fps
    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.follower_port,
            id=args.follower_id,
            cameras=cameras,
            max_relative_target=args.max_relative_target,
        )
    )
    teleop = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    for arm in (robot, teleop):
        if not arm.calibration:
            adopt_calibration(arm)

    robot.connect(calibrate=False)
    teleop.connect(calibrate=False)
    for arm in (robot, teleop):
        if not arm.is_calibrated:
            robot.bus.disable_torque()
            sys.exit(f"{arm}'s motors disagree with calibration {arm.id!r}; run lerobot-calibrate again")

    stack = Stack(robot, teleop, args.fps, args.root, args.owner)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(stack))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    log.info("serving on 127.0.0.1:%d", args.port)

    code = 0
    try:
        stack.run(stopping)
    except Exception:
        log.exception("control loop failed; disconnecting both arms (follower torque off) and exiting")
        code = 1
    finally:
        server.shutdown()
        # LeRobot's disconnect turns the follower's torque off first
        # (disable_torque_on_disconnect), which is the one torque-off a dead bus
        # still gets a chance at.
        for arm in (robot, teleop):
            try:
                arm.disconnect()
            except Exception:
                log.exception("could not disconnect %s cleanly", arm)
    sys.exit(code)


if __name__ == "__main__":
    main()
