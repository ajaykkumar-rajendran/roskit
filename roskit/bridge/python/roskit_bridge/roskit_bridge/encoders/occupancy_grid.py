"""Encoder for OccupancyGrid messages - uses PNG compression."""

import io
import cbor2
import numpy as np
from PIL import Image
from typing import Any, Dict
from .base import BaseEncoder, register_encoder
from ..protocol import Encoding


@register_encoder('nav_msgs/msg/OccupancyGrid')
@register_encoder('OccupancyGrid')
class OccupancyGridEncoder(BaseEncoder):
    """
    Encodes OccupancyGrid as PNG image.

    The grid data is converted to grayscale:
    - -1 (unknown) -> 128 (gray)
    - 0 (free) -> 254 (white)
    - 100 (occupied) -> 0 (black)
    - Values in between are interpolated

    Metadata (resolution, origin, etc.) is sent via CBOR.
    """

    encoding = Encoding.PNG

    def encode(self, msg: Any) -> bytes:
        """Encode occupancy grid data as PNG with metadata prefix."""
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8)

        # Reshape to 2D grid
        grid = data.reshape((height, width))

        # Convert to grayscale image
        # -1 (unknown) -> 128, 0 (free) -> 254, 100 (occupied) -> 0
        img_data = np.zeros((height, width), dtype=np.uint8)

        # Unknown cells
        unknown_mask = grid == -1
        img_data[unknown_mask] = 128

        # Known cells: interpolate from 0 (free=254) to 100 (occupied=0)
        known_mask = grid >= 0
        # Clamp values to 0-100 range
        known_values = np.clip(grid[known_mask], 0, 100)
        # Map 0->254, 100->0
        img_data[known_mask] = (254 - (known_values * 254 / 100)).astype(np.uint8)

        # Flip vertically for correct orientation (ROS uses bottom-left origin)
        img_data = np.flipud(img_data)

        # Create PNG
        image = Image.fromarray(img_data, mode='L')
        buffer = io.BytesIO()
        image.save(buffer, format='PNG', optimize=False)
        image_bytes = buffer.getvalue()

        # Prefix with metadata length + CBOR metadata to align with client expectations
        metadata_bytes = cbor2.dumps(self.get_metadata(msg))
        import struct
        return struct.pack('>I', len(metadata_bytes)) + metadata_bytes + image_bytes

    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """Extract grid metadata."""
        return {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {
                'position': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z,
                },
                'orientation': {
                    'x': msg.info.origin.orientation.x,
                    'y': msg.info.origin.orientation.y,
                    'z': msg.info.origin.orientation.z,
                    'w': msg.info.origin.orientation.w,
                }
            },
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            }
        }

