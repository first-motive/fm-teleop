"""PolicyServer node smoke test — a fake policy, no checkpoint, no robot.

The node is driven through its callbacks and timers directly and its publishers are
replaced with recorders, so the gate → infer → chunk → publish path runs on a bare rclpy
context. A worker thread stands in for the executor's inference thread where the test
is about the two running at once. The trained-checkpoint run stays manual; this is the
deterministic half.
"""

import json
import math
import threading
import time

from control_msgs.msg import JointJog
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from fm_teleop_policy.policies import Policy
from fm_teleop_policy.server import build_executor, PolicyServer

# The default robot_profile + arm, read from the installed profile.
JOINTS = [f"follower_r_joint{index}" for index in range(1, 8)]
WIDTH = len(JOINTS)


class FakePolicy(Policy):
    def __init__(self, chunk=None, error=None):
        self.chunk = chunk if chunk is not None else [[0.1] * WIDTH, [0.3] * WIDTH]
        self.error = error
        self.calls = 0

    def act(self, observation):
        self.calls += 1
        if self.error:
            raise RuntimeError(self.error)
        return self.chunk


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(policy=None, **overrides):
    node = PolicyServer(
        policy=policy or FakePolicy(),
        parameter_overrides=[
            Parameter(name, value=value) for name, value in overrides.items()
        ],
    )
    node._pub = RecordingPublisher()
    node._status_pub = RecordingPublisher()
    return node


def _observation():
    message = JointState()
    message.name = list(JOINTS)
    message.position = [0.0] * WIDTH
    return message


def test_joint_order_defaults_to_the_loader_right_arm(ros):
    node = _make_node()
    try:
        assert node._joints == JOINTS
    finally:
        node.destroy_node()


def test_robot_profile_and_arm_pick_the_joint_order(ros):
    node = _make_node(robot_profile="fm_openarm_v2", arm="left")
    try:
        assert node._joints == [f"openarm_left_joint{index}" for index in range(1, 8)]
    finally:
        node.destroy_node()


def test_an_unknown_robot_profile_is_rejected(ros):
    with pytest.raises(ValueError, match="Unknown robot profile"):
        _make_node(robot_profile="nope")


def test_an_unknown_arm_is_rejected(ros):
    with pytest.raises(ValueError, match="no arm 'middle'"):
        _make_node(arm="middle")


def test_disengaged_by_default_and_silent(ros):
    node = _make_node()
    try:
        node._on_observation(_observation())
        node._infer()
        node._tick()
        assert node._pub.messages == []
        assert node.status()["engaged"] is False
    finally:
        node.destroy_node()


def test_engaged_publishes_the_chunk_one_action_per_tick(ros):
    node = _make_node()
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()
        node._tick()
        node._tick()  # chunk drained — nothing more until the next inference

        assert len(node._pub.messages) == 2
        first = node._pub.messages[0]
        assert first.joint_names == JOINTS
        assert list(first.velocities) == pytest.approx([0.1] * WIDTH)
        assert list(node._pub.messages[1].velocities) == pytest.approx([0.3] * WIDTH)
    finally:
        node.destroy_node()


def test_inference_waits_for_an_observation(ros):
    policy = FakePolicy()
    node = _make_node(policy)
    try:
        node._on_enable(Bool(data=True))
        node._infer()
        assert policy.calls == 0
    finally:
        node.destroy_node()


def test_disengaging_drops_the_buffered_chunk(ros):
    # On re-engage the arm must act on a fresh observation, not on actions inferred
    # before the operator took the arm back.
    node = _make_node()
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._on_enable(Bool(data=False))
        node._on_enable(Bool(data=True))
        node._tick()
        assert node._pub.messages == []
    finally:
        node.destroy_node()


def test_a_stale_chunk_is_dropped_rather_than_replayed(ros):
    node = _make_node(chunk_timeout=0.05)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._chunk_stamp -= 1.0
        node._tick()
        assert node._pub.messages == []
    finally:
        node.destroy_node()


def test_a_wrong_width_action_drops_the_chunk_and_reports(ros):
    node = _make_node(FakePolicy(chunk=[[0.1, 0.2, 0.3]]))
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()
        assert node._pub.messages == []
        assert "3 values" in node.status()["error"]
    finally:
        node.destroy_node()


def test_an_inference_failure_keeps_the_node_up_and_shows_in_status(ros):
    node = _make_node(FakePolicy(error="model exploded"))
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()
        assert node._pub.messages == []
        assert node.status()["error"] == "inference failed: RuntimeError"
    finally:
        node.destroy_node()


def test_a_stale_observation_stops_inference(ros):
    policy = FakePolicy()
    node = _make_node(policy, observation_timeout=0.05)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._observation_stamp -= 1.0
        node._infer()
        node._tick()
        assert policy.calls == 0
        assert node._pub.messages == []
    finally:
        node.destroy_node()


def test_a_fresh_chunk_replaces_the_unpublished_tail(ros):
    # Inference outrunning the drain: freshness wins, because the leftover tail was
    # inferred from an older observation.
    policy = FakePolicy()
    node = _make_node(policy)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()  # one of two actions published
        policy.chunk = [[0.5] * WIDTH]
        node._infer()
        node._tick()
        assert list(node._pub.messages[-1].velocities) == pytest.approx([0.5] * WIDTH)
        assert node._chunk == []
    finally:
        node.destroy_node()


def test_the_drain_outrunning_inference_goes_quiet_not_stale(ros):
    node = _make_node()
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        for _ in range(6):  # more ticks than the chunk has actions
            node._tick()
        assert len(node._pub.messages) == 2
    finally:
        node.destroy_node()


def test_the_observation_is_keyed_by_joint_name(ros):
    node = _make_node()
    try:
        message = _observation()
        message.position = [0.1 * index for index in range(WIDTH)]
        node._on_observation(message)
        assert node._observation == {
            "observation.state": dict(zip(JOINTS, message.position))
        }
    finally:
        node.destroy_node()


def test_status_is_published_as_json(ros):
    node = _make_node()
    try:
        node._publish_status()
        payload = json.loads(node._status_pub.messages[0].data)
        assert payload["policy_type"] == "zero"
        assert payload["engaged"] is False
        assert payload["observing"] is False
        assert payload["robot_profile"] == "loader_openarm_v2"
        assert payload["arm"] == "right"
    finally:
        node.destroy_node()


class BlockingPolicy(Policy):
    """Holds act() open until released, the way a slow model forward pass does."""

    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def act(self, observation):
        self.entered.set()
        assert self.release.wait(timeout=5.0)
        return [[0.5] * WIDTH]


def _start_inference(node, policy):
    worker = threading.Thread(target=node._infer)
    worker.start()
    assert policy.entered.wait(timeout=5.0)
    return worker


def _tick_with_deadline(node):
    # A tick that waits on act() would hang the suite; joining with a timeout turns
    # that regression into a failure.
    ticker = threading.Thread(target=node._tick)
    ticker.start()
    ticker.join(timeout=2.0)
    assert not ticker.is_alive(), "the control tick blocked on inference"


def test_the_control_tick_keeps_publishing_while_inference_is_slow(ros):
    policy = BlockingPolicy()
    node = _make_node(policy)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._chunk = [[0.1] * WIDTH, [0.2] * WIDTH]
        node._chunk_stamp = time.monotonic()
        worker = _start_inference(node, policy)

        # act() is still running on the worker; the tick must neither wait for it
        # nor stop draining the chunk it already has.
        _tick_with_deadline(node)
        _tick_with_deadline(node)
        published = [list(message.velocities) for message in node._pub.messages]
        assert published == [pytest.approx([0.1] * WIDTH), pytest.approx([0.2] * WIDTH)]

        policy.release.set()
        worker.join(timeout=5.0)
        node._tick()
        assert list(node._pub.messages[-1].velocities) == pytest.approx([0.5] * WIDTH)
    finally:
        policy.release.set()
        node.destroy_node()


def test_a_chunk_landing_after_the_observations_went_quiet_is_not_published(ros):
    policy = BlockingPolicy()
    node = _make_node(policy, observation_timeout=0.05)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        worker = _start_inference(node, policy)
        time.sleep(0.1)  # no observation arrives while act() runs
        policy.release.set()
        worker.join(timeout=5.0)
        node._tick()
        assert node._pub.messages == []
        assert node._chunk == []
    finally:
        policy.release.set()
        node.destroy_node()


def test_an_inference_failure_drops_the_buffered_chunk(ros):
    policy = FakePolicy(chunk=[[0.1] * WIDTH, [0.2] * WIDTH, [0.3] * WIDTH])
    node = _make_node(policy)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()
        policy.error = "model exploded"
        node._infer()
        node._tick()
        assert len(node._pub.messages) == 1
    finally:
        node.destroy_node()


def test_the_executor_keeps_the_tick_running_while_act_is_blocked(ros):
    # Spins the real executor wiring: under a single-threaded executor, or with the
    # inference timer in the default callback group, the held act() starves the tick.
    policy = BlockingPolicy()
    node = _make_node(policy, chunk_timeout=10.0, observation_timeout=10.0)
    executor = build_executor(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        with node._lock:
            node._chunk = [[0.1] * WIDTH] * 500
            node._chunk_stamp = time.monotonic()
        spinner.start()
        assert policy.entered.wait(timeout=5.0)
        before = len(node._pub.messages)
        deadline = time.monotonic() + 2.0
        while len(node._pub.messages) < before + 5 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(node._pub.messages) >= before + 5
        assert all(isinstance(message, JointJog) for message in node._pub.messages)
    finally:
        policy.release.set()
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()


def test_a_chunk_inferred_across_a_disengage_is_discarded(ros):
    # The operator took the arm back mid-inference; that chunk was computed for an
    # engagement that has ended, even if the gate is open again by the time it lands.
    policy = BlockingPolicy()
    node = _make_node(policy)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        worker = _start_inference(node, policy)
        node._on_enable(Bool(data=False))
        node._on_enable(Bool(data=True))
        policy.release.set()
        worker.join(timeout=5.0)
        node._tick()
        assert node._pub.messages == []
    finally:
        policy.release.set()
        node.destroy_node()


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_a_non_finite_action_is_dropped_not_published(ros, bad):
    action = [0.1] * WIDTH
    action[3] = bad
    node = _make_node(FakePolicy(chunk=[action, [0.1] * WIDTH]))
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()
        node._tick()
        assert node._pub.messages == []
        assert "non-finite" in node.status()["error"]
    finally:
        node.destroy_node()


@pytest.mark.parametrize("chunk", ["nope", 7, [0.1] * WIDTH, [["a"] * WIDTH]])
def test_malformed_policy_output_stores_no_chunk_and_keeps_the_node_up(ros, chunk):
    node = _make_node(FakePolicy(chunk=chunk))
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=True))
        node._infer()
        node._tick()
        assert node._pub.messages == []
        assert node._chunk == []
        assert node.status()["error"]
    finally:
        node.destroy_node()


def test_status_names_the_checkpoint_by_basename_only(ros):
    node = _make_node(checkpoint="/home/operator/runs/act_can/")
    try:
        assert node.status()["checkpoint"] == "act_can"
    finally:
        node.destroy_node()


SIM_ENGAGE = {"engage_on_start": True, "use_sim_time": True, "sim_only": True}


def test_engage_on_start_engages_on_the_first_observation_in_sim(ros):
    node = _make_node(**SIM_ENGAGE)
    try:
        assert node.status()["engaged"] is False
        node._on_observation(_observation())
        assert node.status()["engaged"] is True
        node._infer()
        node._tick()
        assert len(node._pub.messages) == 1
    finally:
        node.destroy_node()


def test_engage_on_start_engages_once_so_a_disengage_holds(ros):
    node = _make_node(**SIM_ENGAGE)
    try:
        node._on_observation(_observation())
        node._on_enable(Bool(data=False))
        node._on_observation(_observation())
        assert node.status()["engaged"] is False
    finally:
        node.destroy_node()


def _partial_observation():
    message = _observation()
    message.name = message.name[:-1]
    message.position = message.position[:-1]
    return message


def _non_finite_observation():
    message = _observation()
    message.position[3] = math.nan
    return message


@pytest.mark.parametrize("invalid", [_partial_observation, _non_finite_observation])
def test_engage_on_start_waits_for_a_complete_finite_observation(ros, invalid):
    node = _make_node(**SIM_ENGAGE)
    try:
        node._on_observation(invalid())
        assert node.status()["engaged"] is False
        node._on_observation(_observation())
        assert node.status()["engaged"] is True
    finally:
        node.destroy_node()


@pytest.mark.parametrize("missing", ["use_sim_time", "sim_only"])
def test_engage_on_start_is_refused_without_both_sim_guards(ros, missing):
    node = _make_node(**{**SIM_ENGAGE, missing: False})
    try:
        node._on_observation(_observation())
        node._infer()
        node._tick()
        assert node.status()["engaged"] is False
        assert node._pub.messages == []
    finally:
        node.destroy_node()


class _HookedLock:
    """The node's lock, running ``hook`` once right after a release."""

    def __init__(self, lock):
        self.lock = lock
        self.hook = None

    def __enter__(self):
        return self.lock.__enter__()

    def __exit__(self, *exc):
        self.lock.__exit__(*exc)
        hook, self.hook = self.hook, None
        if hook:
            hook()


def test_a_disengage_landing_right_after_the_one_shot_holds(ros):
    node = _make_node(**SIM_ENGAGE)
    try:
        lock = node._lock = _HookedLock(node._lock)
        # The operator's disengage arrives as soon as the observation releases the
        # lock: any engage the one-shot did after that release would overrule it.
        lock.hook = lambda: node._on_enable(Bool(data=False))
        node._on_observation(_observation())
        assert node.status()["engaged"] is False
        node._on_observation(_observation())
        assert node.status()["engaged"] is False
    finally:
        node.destroy_node()


def test_an_operator_gate_message_cancels_a_pending_one_shot(ros):
    node = _make_node(**SIM_ENGAGE)
    try:
        node._on_enable(Bool(data=False))
        node._on_observation(_observation())
        assert node.status()["engaged"] is False
    finally:
        node.destroy_node()
