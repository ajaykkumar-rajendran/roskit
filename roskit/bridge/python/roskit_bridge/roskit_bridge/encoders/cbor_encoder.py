"""CBOR encoder for structured ROS messages."""

import cbor2
from typing import Any, Dict
from .base import BaseEncoder
from ..protocol import Encoding


def ros_msg_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert a ROS message to a dictionary recursively."""
    if hasattr(msg, 'get_fields_and_field_types'):
        # ROS2 message
        result = {}
        for field_name in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field_name)
            result[field_name] = ros_msg_to_dict(value)
        return result
    elif isinstance(msg, (list, tuple)):
        return [ros_msg_to_dict(item) for item in msg]
    elif isinstance(msg, bytes):
        # Keep bytes as-is for CBOR
        return msg
    elif hasattr(msg, '__iter__') and not isinstance(msg, (str, bytes)):
        # Handle numpy arrays and other iterables
        try:
            return list(msg)
        except:
            return msg
    else:
        return msg


class CborEncoder(BaseEncoder):
    """Default CBOR encoder for structured messages."""

    encoding = Encoding.CBOR

    def encode(self, msg: Any) -> bytes:
        """Encode message to CBOR."""
        data = ros_msg_to_dict(msg)
        return cbor2.dumps(data)

    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """No separate metadata for CBOR - all in payload."""
        return {}
