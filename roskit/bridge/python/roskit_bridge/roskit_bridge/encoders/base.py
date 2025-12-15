"""Base encoder class and registry."""

from abc import ABC, abstractmethod
from typing import Any, Dict, Type
from ..protocol import Encoding


class BaseEncoder(ABC):
    """Base class for message encoders."""

    encoding: Encoding = Encoding.CBOR

    @abstractmethod
    def encode(self, msg: Any) -> bytes:
        """Encode a ROS message to bytes."""
        pass

    @abstractmethod
    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """Extract metadata from message (sent separately via CBOR)."""
        pass


# Registry of encoders by message type
_encoder_registry: Dict[str, Type[BaseEncoder]] = {}


def register_encoder(msg_type: str):
    """Decorator to register an encoder for a message type."""
    def decorator(cls: Type[BaseEncoder]):
        _encoder_registry[msg_type] = cls
        return cls
    return decorator


def get_encoder(msg_type: str) -> BaseEncoder:
    """Get an encoder instance for a message type."""
    # Check for exact match
    if msg_type in _encoder_registry:
        return _encoder_registry[msg_type]()

    # Check for partial match (e.g., "OccupancyGrid" matches "nav_msgs/msg/OccupancyGrid")
    for registered_type, encoder_cls in _encoder_registry.items():
        if msg_type.endswith(registered_type) or registered_type in msg_type:
            return encoder_cls()

    # Default to CBOR encoder
    from .cbor_encoder import CborEncoder
    return CborEncoder()
