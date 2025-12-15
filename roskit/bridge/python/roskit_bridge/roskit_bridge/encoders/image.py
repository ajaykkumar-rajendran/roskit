"""Encoder for Image messages - uses JPEG/PNG compression."""

import io
import cbor2
import struct
import numpy as np
from PIL import Image
from typing import Any, Dict
from .base import BaseEncoder, register_encoder
from ..protocol import Encoding


@register_encoder('sensor_msgs/msg/Image')
@register_encoder('Image')
class ImageEncoder(BaseEncoder):
    """
    Encodes Image as JPEG or PNG.

    JPEG is used for color images (lossy but much smaller).
    PNG is used when lossless is needed.
    """

    encoding = Encoding.JPEG

    def __init__(self, quality: int = 75, use_png: bool = False):
        self.quality = quality
        self.use_png = use_png
        self.encoding = Encoding.PNG if use_png else Encoding.JPEG

    def encode(self, msg: Any) -> bytes:
        """Encode image as JPEG/PNG."""
        metadata = self.get_metadata(msg)
        metadata_bytes = cbor2.dumps(metadata)

        # Convert ROS image to numpy array
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        # Handle different encodings
        if encoding in ('rgb8', 'RGB8'):
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
            mode = 'RGB'
        elif encoding in ('bgr8', 'BGR8'):
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
            img_array = img_array[:, :, ::-1]  # BGR to RGB
            mode = 'RGB'
        elif encoding in ('rgba8', 'RGBA8'):
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 4))
            mode = 'RGBA'
        elif encoding in ('bgra8', 'BGRA8'):
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 4))
            img_array = img_array[:, :, [2, 1, 0, 3]]  # BGRA to RGBA
            mode = 'RGBA'
        elif encoding in ('mono8', 'MONO8', '8UC1'):
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
            mode = 'L'
        elif encoding in ('mono16', 'MONO16', '16UC1'):
            img_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
            # Scale to 8-bit
            img_array = (img_array / 256).astype(np.uint8)
            mode = 'L'
        else:
            # Try to handle as raw bytes
            try:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, -1))
                if img_array.shape[2] == 3:
                    mode = 'RGB'
                elif img_array.shape[2] == 4:
                    mode = 'RGBA'
                else:
                    mode = 'L'
                    img_array = img_array[:, :, 0]
            except:
                raise ValueError(f"Unsupported image encoding: {encoding}")

        # Create PIL image
        image = Image.fromarray(img_array, mode=mode)

        # Compress
        buffer = io.BytesIO()
        if self.use_png:
            image.save(buffer, format='PNG', optimize=False)
        else:
            # Convert RGBA to RGB for JPEG (no alpha support)
            if mode == 'RGBA':
                image = image.convert('RGB')
            image.save(buffer, format='JPEG', quality=self.quality, optimize=False)

        image_bytes = buffer.getvalue()

        # Pack: metadata_length + metadata + image
        return struct.pack('>I', len(metadata_bytes)) + metadata_bytes + image_bytes

    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """Extract image metadata."""
        return {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'height': msg.height,
            'width': msg.width,
            'encoding': msg.encoding,
            'is_bigendian': msg.is_bigendian,
            'step': msg.step,
        }


@register_encoder('sensor_msgs/msg/CompressedImage')
@register_encoder('CompressedImage')
class CompressedImageEncoder(BaseEncoder):
    """
    Passthrough encoder for CompressedImage - already compressed.
    """

    encoding = Encoding.JPEG  # Will be overridden based on format

    def encode(self, msg: Any) -> bytes:
        """Pass through compressed image data."""
        metadata = self.get_metadata(msg)
        metadata_bytes = cbor2.dumps(metadata)

        # The data is already compressed
        image_bytes = bytes(msg.data)

        return struct.pack('>I', len(metadata_bytes)) + metadata_bytes + image_bytes

    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """Extract compressed image metadata."""
        return {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'format': msg.format,
        }
