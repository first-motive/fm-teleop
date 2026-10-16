"""Policy backends — checkpoint in, action chunk out, no ROS.

Carried over from fm-policy's frozen ``fm_policy_serve`` (tag ``v0.1.2``), which this
package succeeds; that package is not edited, so the two copies do not share code.

The serve node's job is plumbing: read observations off the graph, hand them to a policy,
put the actions back on the graph. *What* the policy is stays behind this one interface,
so a new model family is a new class here and nothing in the node changes:

    load_policy(kind, checkpoint) -> Policy
    Policy.act(observation) -> [action, ...]     one chunk, oldest action first

``act`` returns a *chunk* rather than one action because trained models predict a horizon
(GR00T's config declares ``action_horizon: 40``): the node buffers the chunk and publishes
from it at its own rate, so inference can run slower than control without the arm
stuttering. A one-element chunk is a valid answer — LeRobot's ``select_action`` queues a
policy's horizon internally and hands out one step at a time, so that backend returns one.

Two backends ship:

    zero      no checkpoint, all-zero actions — proves the loop end to end in sim
    lerobot   a LeRobot/GR00T checkpoint directory, loaded lazily

``lerobot`` imports torch and lerobot inside ``load()``, never at module import: those
live in their own venv rather than the ROS environment, so the node, its tests, and the
``zero`` path must all work without them.
"""

import math
import numbers
import os


def check_action(action, width):
    """Return why ``action`` cannot drive ``width`` joints, or None when it can."""
    if not isinstance(action, (list, tuple)):
        return f"action is a {type(action).__name__}, not a list of numbers"
    if len(action) != width:
        return f"policy returned {len(action)} values for {width} joints"
    # bool is a numbers.Real, but True is not a velocity.
    if not all(
        isinstance(value, numbers.Real) and not isinstance(value, bool) for value in action
    ):
        return "action holds a value that is not a real number"
    if not all(math.isfinite(value) for value in action):
        # A NaN or inf velocity is not a request Servo can be trusted to refuse.
        return "policy returned a non-finite action"
    return None


class Policy:
    """The interface the serve node depends on."""

    #: Names of the action vector's elements, or None when the backend does not know them.
    action_names = None

    def act(self, observation):
        """Return an action chunk for one observation: a list of action vectors."""
        raise NotImplementedError


class ZeroPolicy(Policy):
    """Emit a zero action of a fixed width.

    Not a stand-in for a model — a way to prove the whole serve loop (observations in,
    actions on the contract's channel, arbiter gate, status) without a trained
    checkpoint, which is what makes the sim-first CI run possible.
    """

    def __init__(self, action_names):
        self.action_names = list(action_names)

    def act(self, observation):
        return [[0.0] * len(self.action_names)]


class LeRobotPolicy(Policy):
    """A LeRobot-format checkpoint directory, loaded through LeRobot's own classes.

    The checkpoint is a HuggingFace-style directory (config + weights + statistics), not a
    bare tensor file, so ``checkpoint`` names the directory. ``action_names`` is the joint
    order the observation vector is built in and the action vector is read back in — the
    follower's controller order, which the node reads from the robot profile.

    tradeoff: state-only observations. LeRobot's ``select_action`` takes a feature batch,
    and this backend fills ``observation.state`` alone, so a policy trained on camera
    features will reject the batch with a missing-key error rather than act on half its
    input. Wiring images is the next step and belongs with the recorder's camera topics.
    """

    def __init__(self, checkpoint, action_names=None):
        self.checkpoint = checkpoint
        self.action_names = list(action_names) if action_names else None
        self._policy = None
        self._pre = None
        self._post = None
        self._device = "cpu"

    def load(self):
        """Import LeRobot and instantiate the policy. Raises if either is unavailable."""
        if not os.path.isdir(self.checkpoint):
            raise FileNotFoundError(
                f"Checkpoint directory not found: {self.checkpoint}. A LeRobot checkpoint "
                "is a directory (config + weights), not a single file."
            )
        if not self.action_names:
            raise ValueError(
                "A lerobot policy needs action_names — the joint order its state and "
                "action vectors are in."
            )
        try:
            from lerobot.configs.policies import PreTrainedConfig
            from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot is not importable in this environment. Serve a 'lerobot' policy "
                "from an environment that has it, or use policy_type 'zero' to exercise "
                "the loop without a checkpoint."
            ) from exc
        config = PreTrainedConfig.from_pretrained(self.checkpoint)
        # from_pretrained puts the policy on its config's device and in eval mode.
        self._policy = get_policy_class(config.type).from_pretrained(self.checkpoint)
        # The processors carry the normalisation the checkpoint was trained with. Calling
        # select_action on raw joint positions would feed the model unnormalised input and
        # read back unnormalised actions — silently wrong rather than an error.
        self._pre, self._post = make_pre_post_processors(config, self.checkpoint)
        self._device = config.device
        return self

    def act(self, observation):
        if self._policy is None:
            raise RuntimeError("LeRobotPolicy.load() must run before act().")
        import torch

        state = observation["observation.state"]
        try:
            vector = [float(state[name]) for name in self.action_names]
        except KeyError as exc:
            raise ValueError(
                f"Observation is missing joint {exc} of {self.action_names}."
            ) from exc
        batch = {
            "observation.state": torch.tensor(
                [vector], dtype=torch.float32, device=self._device
            )
        }
        with torch.inference_mode():
            action = self._post(self._policy.select_action(self._pre(batch)))
        # select_action returns one action per call — a chunked policy queues its horizon
        # internally and hands the next step out — so a one-element chunk is the honest
        # shape here. The node's buffer handles longer chunks either way.
        return [[float(value) for value in action.squeeze(0).tolist()]]


def load_policy(kind, checkpoint, action_names=None):
    """Build and load the named backend.

    ``zero`` ignores ``checkpoint`` and needs ``action_names`` for its width. Raises
    ValueError for an unknown kind, naming the ones that exist.
    """
    if kind == "zero":
        if not action_names:
            raise ValueError("policy_type 'zero' needs action_names to size its output.")
        return ZeroPolicy(action_names)
    if kind == "lerobot":
        if not checkpoint:
            raise ValueError("policy_type 'lerobot' needs a checkpoint directory.")
        return LeRobotPolicy(checkpoint, action_names).load()
    raise ValueError(f"Unknown policy_type '{kind}'. One of: zero, lerobot.")
