"""TeleopSource — the base node every teleop input subclasses.

The contract (``contract.py``) says *what* a source may publish; this base says *how*,
so a new source is one file of device-to-command mapping logic and nothing else. It
gives each subclass two things:

    contract_publisher(channel)  a publisher already typed + topic'd from the contract
    stamped_header(frame_id)     a Header stamped with the node clock, for *Stamped msgs

A subclass declares its parameters, builds the publishers it needs in ``__init__`` via
``contract_publisher``, subscribes to its device, and maps each device reading onto a
contract message in the callback. It never hard-codes a topic or message type — those
come from the contract, the single point that the sinks in ``fm_control`` agree on.
"""

from rclpy.node import Node
from std_msgs.msg import Header

from fm_teleop_core import contract


class TeleopSource(Node):
    """Base node binding a teleop source to the shared command contract."""

    def __init__(self, node_name):
        super().__init__(node_name)

    def contract_publisher(self, channel, topic=None, qos=10):
        """Create a publisher for a contract channel.

        ``channel`` is a channel key (e.g. ``"arm_twist"``) or a ``Channel``. ``topic``
        overrides the channel default and is required for channels whose default topic
        is empty (the per-side hand channels, the per-arm leader trajectory).
        """
        ch = channel if isinstance(channel, contract.Channel) else contract.channel(channel)
        resolved = topic or ch.topic
        if not resolved:
            raise ValueError(
                f"Channel '{ch.key}' has no default topic; pass an explicit topic."
            )
        return self.create_publisher(ch.msg_type, resolved, qos)

    def stamped_header(self, frame_id):
        """Return a Header stamped now in ``frame_id``, for the *Stamped messages."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header
