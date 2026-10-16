"""Policy serving node — a trained checkpoint drives the same arm the operator drives.

    /joint_states  ->  observation  ->  policy.act  ->  action chunk
                                                    ->  arm_joint (JointJog) -> Servo

Successor to fm-policy's frozen ``fm_policy_serve`` (tag ``v0.1.2``); the action vector's
joint order comes from a robot profile (``robot_profile`` + ``arm``). The node owns the
ROS plumbing only; models live behind ``policies.load_policy``. Inference runs in its own
callback group under ``build_executor``'s executor, so a slow ``act()`` never stalls the
control tick. The engage gate and the two timeouts are described in the package README.
"""

import json
import os
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import JointJog
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from fm_teleop_core import contract
from fm_teleop_core.source import TeleopSource
from fm_teleop_policy.policies import check_action, load_policy
from fm_teleop_policy.profile import arm_joints, load_profile


class PolicyServer(TeleopSource):
    """Run a checkpoint against live observations and publish actions behind the gate."""

    def __init__(self, *, policy=None, **kwargs):
        super().__init__("policy_server", **kwargs)

        self._profile_name = self._declare(
            "robot_profile", "loader_openarm_v2",
            "Robot profile whose joint names the policy acts in.")
        self._arm = self._declare(
            "arm", "right", "Which arm of the robot profile the policy drives.")
        self._policy_type = self._declare(
            "policy_type", "zero",
            "Policy backend: zero (no checkpoint, proves the loop) | lerobot.")
        self._checkpoint = self._declare(
            "checkpoint", "", "Checkpoint directory for a lerobot policy.")

        observation_topic = self._declare(
            "observation_topic", "/joint_states", "Observation source.")
        command_topic = self._declare(
            "command_topic", contract.ARM_JOINT.topic, "The contract's arm_joint channel.")
        enable_topic = self._declare(
            "enable_topic", "~/enable", "The operator-held engage gate (Bool).")
        status_topic = self._declare(
            "status_topic", "~/status", "JSON status for a desktop autonomy rail.")

        rate_hz = float(self._declare(
            "rate_hz", 50.0, "Control cadence: drains the action chunk."))
        inference_rate_hz = float(self._declare(
            "inference_rate_hz", 10.0, "Model cadence: fills the action chunk."))
        status_rate_hz = float(self._declare(
            "status_rate_hz", 1.0, "Status line cadence."))
        self._chunk_timeout = float(self._declare(
            "chunk_timeout", 1.0,
            "Seconds an unrefilled chunk stays publishable before the node goes quiet."))
        self._observation_timeout = float(self._declare(
            "observation_timeout", 0.5,
            "Seconds without an observation before inference and publishing stop."))

        profile = load_profile(self._profile_name)
        self._joints = arm_joints(profile, self._arm)

        self._policy = policy or load_policy(
            self._policy_type, self._checkpoint, self._joints
        )

        self._pub = self.contract_publisher("arm_joint", topic=command_topic)
        self._status_pub = self.create_publisher(String, status_topic, 10)

        # Inference runs on its own thread (see build_executor), so everything below is
        # read and written under this lock. _engagement counts engage transitions: a
        # chunk whose inference straddled one was computed for a gate state that no
        # longer holds.
        self._lock = threading.Lock()
        self._engagement = 0
        self._engaged = False
        self._observation = None
        self._observation_stamp = None
        self._chunk = []
        self._chunk_stamp = None
        self._inference_seconds = None
        self._error = None

        self.create_subscription(JointState, observation_topic, self._on_observation, 10)
        self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.create_timer(
            1.0 / inference_rate_hz,
            self._infer,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(1.0 / status_rate_hz, self._publish_status)

        self.get_logger().info(
            f"policy_server up: {self._policy_type} policy over "
            f"{profile['name']} {self._arm} arm ({len(self._joints)} joints) -> "
            f"{command_topic}, gated on {enable_topic} (disengaged)"
        )

    def _declare(self, name, default, description):
        # Every parameter is read once here, so a runtime set would be accepted and
        # silently ignored; read_only refuses it instead.
        return self.declare_parameter(
            name,
            default,
            ParameterDescriptor(description=description, read_only=True),
        ).value

    def _observation_stale(self, now):
        return now - self._observation_stamp > self._observation_timeout

    def _on_observation(self, msg):
        observation = {"observation.state": dict(zip(msg.name, msg.position))}
        with self._lock:
            self._observation = observation
            self._observation_stamp = time.monotonic()

    def _on_enable(self, msg):
        engaged = bool(msg.data)
        with self._lock:
            changed = engaged != self._engaged
            if changed:
                self._engagement += 1
            if not engaged:
                # Drop the buffered chunk: on re-engage the arm must act on a fresh
                # observation, not on actions inferred before the operator took over.
                self._chunk = []
            self._engaged = engaged
        if changed:
            self.get_logger().info(
                f"policy_server: {'engaged' if engaged else 'disengaged'}."
            )

    def _infer(self):
        with self._lock:
            if not self._engaged or self._observation is None:
                return
            if self._observation_stale(time.monotonic()):
                self._chunk = []
                return
            observation = self._observation
            engagement = self._engagement
        started = time.monotonic()
        # A model fault, or output that is not a chunk at all, must not take the node
        # down mid-session.
        try:
            chunk = list(self._policy.act(observation))
            error = next(
                (
                    problem
                    for problem in (check_action(a, len(self._joints)) for a in chunk)
                    if problem
                ),
                None,
            )
        except Exception as exc:
            self.get_logger().error(f"policy_server: inference failed ({exc!r}).")
            # Status leaves the node, so it carries the class only, not the message.
            error = f"inference failed: {type(exc).__name__}"
        finished = time.monotonic()
        with self._lock:
            self._inference_seconds = finished - started
            self._error = error
            if error:
                # The buffered tail came from the model that just failed.
                self._chunk = []
            elif engagement == self._engagement:
                # A fresh chunk replaces whatever is left of the last one. Freshness
                # wins: an unpublished tail was inferred from an older observation.
                self._chunk = [list(action) for action in chunk]
                self._chunk_stamp = finished
        if error:
            self.get_logger().error(f"policy_server: {error}; no chunk stored.")

    def _tick(self):
        with self._lock:
            if not self._engaged or not self._chunk:
                return
            now = time.monotonic()
            if now - self._chunk_stamp > self._chunk_timeout or self._observation_stale(now):
                # Checking the observation here, not only before act(), covers a chunk
                # that landed after the state stream went quiet during inference.
                self._chunk = []
                return
            action = self._chunk.pop(0)
            error = check_action(action, len(self._joints))
            if error:
                # The rest of the chunk came from the same faulty inference step.
                self._error = error
                self._chunk = []
        if error:
            self.get_logger().error(f"policy_server: {error}; dropping the chunk.")
            return
        message = JointJog()
        message.header = self.stamped_header("")
        message.joint_names = list(self._joints)
        message.velocities = [float(value) for value in action]
        self._pub.publish(message)

    def _publish_status(self):
        """Publish one JSON status line for the desktop's autonomy rail."""
        self._status_pub.publish(String(data=json.dumps(self.status())))

    def status(self):
        """Return the status payload — the node's own view of what it is doing."""
        with self._lock:
            engaged = self._engaged
            observing = self._observation is not None
            buffered = len(self._chunk)
            inference_seconds = self._inference_seconds
            error = self._error
        return {
            "policy_type": self._policy_type,
            # The basename names the checkpoint without publishing the host's layout.
            "checkpoint": os.path.basename(self._checkpoint.rstrip("/")),
            "robot_profile": self._profile_name,
            "arm": self._arm,
            "engaged": engaged,
            "observing": observing,
            "buffered_actions": buffered,
            "inference_ms": (
                round(inference_seconds * 1000.0, 1)
                if inference_seconds is not None
                else None
            ),
            "error": error,
        }


def build_executor(node):
    """Return the executor the node must spin in for inference to run off the tick."""
    # Two threads are enough: inference holds one while the control tick, the
    # subscriptions, and the status timer share the other.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    return executor


def main(args=None):
    rclpy.init(args=args)
    node = PolicyServer()
    executor = build_executor(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
