"""The stack through its loopback API, with LeRobot's real loop and dataset writer.

Only the arms are stand-ins: the buses are hardware, the one boundary a CI run
cannot cross. Everything between the HTTP request and the v3 files on disk is
the code that runs on a robot host.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest
from lerobot.datasets import LeRobotDataset
from lerobot.teleoperators import Teleoperator

from fm_teleop_so101.__main__ import handler_for
from fm_teleop_so101.stack import Stack

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class Bus:
    def __init__(self) -> None:
        self.torque = True  # LeRobot's connect() leaves the follower enabled

    def enable_torque(self) -> None:
        self.torque = True

    def disable_torque(self) -> None:
        self.torque = False


class Follower:
    name = "so101_follower"

    def __init__(self) -> None:
        self.cameras: dict = {}
        self.action_features = {f"{j}.pos": float for j in JOINTS}
        self.observation_features = {f"{j}.pos": float for j in JOINTS}
        self.bus = Bus()
        self.sent = 0

    def get_observation(self) -> dict:
        return {f"{j}.pos": float(i) for i, j in enumerate(JOINTS)}

    def send_action(self, action: dict) -> dict:
        self.sent += 1
        return action


class Leader:
    def get_action(self) -> dict:
        return {f"{j}.pos": 1.0 for j in JOINTS}


# record_loop drives a teleop only if it is a Teleoperator; otherwise it spins
# without sleeping. The real SO101Leader is one.
Teleoperator.register(Leader)


class Rig:
    def __init__(self, root) -> None:
        self.root = root
        self.robot = Follower()
        self.stack = Stack(self.robot, Leader(), 30, root, "fm-rob-03")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(self.stack))
        self.port = self.server.server_address[1]
        self.stopping = threading.Event()
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.worker = threading.Thread(target=self.stack.run, args=(self.stopping,), daemon=True)
        self.worker.start()

    def call(self, method, path, body=None, headers=None) -> tuple[int, dict]:
        request = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}",
            data=json.dumps(body).encode() if body is not None else None,
            method=method,
            headers={"Content-Type": "application/json", **(headers or {})},
        )
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as refusal:
            return refusal.code, json.loads(refusal.read())

    def status(self) -> dict:
        return self.call("GET", "/status")[1]

    def until(self, check, what: str, timeout_s: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if check():
                return
            time.sleep(0.05)
        raise AssertionError(f"timed out waiting for {what}; status {self.status()}")

    def teleop(self) -> None:
        assert self.call("POST", "/mode", {"mode": "teleop"})[0] == 200
        self.until(lambda: self.status()["mode"] == "teleop" and self.robot.sent > 3, "teleop")

    def take(self, dataset="pick-cube", task="pick up the cube", seconds=0.5) -> dict:
        assert self.call("POST", "/record", {"action": "start", "dataset": dataset, "task": task})[0] == 200
        self.until(lambda: self.status()["mode"] == "recording", "the take to start")
        time.sleep(seconds)
        before = self.status()["last_episode"]
        assert self.call("POST", "/record", {"action": "stop"})[0] == 200
        self.until(lambda: self.status()["last_episode"] != before, "the take to save")
        return self.status()["last_episode"]

    def close(self) -> None:
        self.stopping.set()
        self.call("POST", "/stop")
        self.worker.join(10)
        self.server.shutdown()


@pytest.fixture
def rig(tmp_path):
    rig = Rig(tmp_path / "lerobot")
    rig.until(lambda: rig.status()["mode"] == "idle", "idle")
    yield rig
    rig.close()


def test_the_stack_starts_idle_with_torque_off_and_live_joints(rig):
    status = rig.status()
    assert status["mode"] == "idle"
    assert not rig.robot.bus.torque, "an idle follower must not hold torque"
    assert sorted(status["joints"]) == sorted(JOINTS)


def test_teleop_turns_torque_on_and_commands_the_follower(rig):
    rig.teleop()
    assert rig.robot.bus.torque


def test_recording_never_starts_motion_by_itself(rig):
    code, body = rig.call("POST", "/record", {"action": "start", "dataset": "pick", "task": "t"})
    assert code == 409 and "teleop first" in body["error"]
    assert rig.status()["mode"] == "idle"


@pytest.mark.parametrize("dataset", ["../etc", "a/b", "", ".hidden", "x" * 65])
def test_a_dataset_name_that_could_become_a_path_is_refused(rig, dataset):
    rig.teleop()
    assert rig.call("POST", "/record", {"action": "start", "dataset": dataset, "task": "t"})[0] == 409


def test_a_take_without_a_task_sentence_is_refused(rig):
    rig.teleop()
    assert rig.call("POST", "/record", {"action": "start", "dataset": "pick", "task": "  "})[0] == 409


def test_an_unknown_mode_is_refused(rig):
    assert rig.call("POST", "/mode", {"mode": "dance"})[0] == 409


def test_stopping_with_no_take_is_refused(rig):
    assert rig.call("POST", "/record", {"action": "stop"})[0] == 409


def test_a_take_cannot_start_twice_or_be_left_by_idling(rig):
    rig.teleop()
    rig.call("POST", "/record", {"action": "start", "dataset": "pick-cube", "task": "pick up the cube"})
    rig.until(lambda: rig.status()["mode"] == "recording", "the take to start")
    assert rig.call("POST", "/record", {"action": "start", "dataset": "pick-cube", "task": "t"})[0] == 409
    assert rig.call("POST", "/mode", {"mode": "idle"})[0] == 409


def test_takes_become_episodes_of_one_v3_dataset_with_their_task(rig):
    rig.teleop()
    first = rig.take()
    second = rig.take()
    assert (first["episode"], second["episode"]) == (0, 1), "the second take appends to the dataset"
    assert rig.status()["mode"] == "teleop", "a finished take returns to teleop"
    dataset = LeRobotDataset("fm-rob-03/pick-cube", root=rig.root / "pick-cube")
    assert dataset.meta.total_episodes == 2
    assert list(dataset.meta.tasks.index) == ["pick up the cube"]
    assert dataset.meta.total_frames > 0


def test_stop_saves_the_open_take_and_leaves_the_follower_limp(rig):
    rig.teleop()
    rig.call("POST", "/record", {"action": "start", "dataset": "pick-cube", "task": "pick up the cube"})
    rig.until(lambda: rig.status()["mode"] == "recording", "the take to start")
    time.sleep(0.5)
    assert rig.call("POST", "/stop")[0] == 200
    rig.until(lambda: rig.status()["last_episode"] is not None, "the interrupted take to save")
    rig.until(lambda: rig.status()["mode"] == "idle", "idle")
    assert not rig.robot.bus.torque


@pytest.mark.parametrize(
    "headers,code",
    [
        ({"Origin": "https://example.com"}, 403),
        ({"Host": "rebound.example:8790"}, 403),
        ({"Content-Type": "text/plain"}, 415),
    ],
)
def test_a_request_a_web_page_could_send_is_refused(rig, headers, code):
    assert rig.call("POST", "/mode", {"mode": "teleop"}, headers)[0] == code
    assert rig.status()["mode"] == "idle"


def test_an_oversized_body_is_refused(rig):
    assert rig.call("POST", "/mode", {"mode": "x" * 5000})[0] == 400
