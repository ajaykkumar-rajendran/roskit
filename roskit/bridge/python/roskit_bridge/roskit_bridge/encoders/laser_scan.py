"""Encoder for LaserScan messages - uses binary float arrays."""

import struct
import cbor2
import numpy as np
from typing import Any, Dict
from .base import BaseEncoder, register_encoder
from ..protocol import Encoding


@register_encoder('sensor_msgs/msg/LaserScan')
@register_encoder('LaserScan')
class LaserScanEncoder(BaseEncoder):
    """
    Encodes LaserScan with binary float arrays for ranges/intensities.

    This is much more efficient than JSON/CBOR for large arrays of floats.
    A typical 720-point scan:
    - JSON: ~7KB
    - CBOR: ~3KB
    - Binary: ~2.9KB (raw floats, no overhead)

    Format:
    [metadata_length (4 bytes)]
    [metadata (CBOR)]
    [ranges (float32 array)]
    [intensities (float32 array)]  # if present
    """

    encoding = Encoding.BINARY

    def encode(self, msg: Any) -> bytes:
        """Encode laser scan as binary."""
        metadata = self.get_metadata(msg)
        metadata_bytes = cbor2.dumps(metadata)

        # Convert ranges to float32 array
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges_bytes = ranges.tobytes()

        # Convert intensities if present
        intensities_bytes = b''
        if len(msg.intensities) > 0:
            intensities = np.array(msg.intensities, dtype=np.float32)
            intensities_bytes = intensities.tobytes()

        # Pack: metadata_length + metadata + ranges + intensities
        return struct.pack('>I', len(metadata_bytes)) + metadata_bytes + ranges_bytes + intensities_bytes

    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """Extract scan metadata."""
        return {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'time_increment': float(msg.time_increment),
            'scan_time': float(msg.scan_time),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max),
            'ranges_count': len(msg.ranges),
            'intensities_count': len(msg.intensities),
        }


@register_encoder('sensor_msgs/msg/PointCloud2')
@register_encoder('PointCloud2')
class PointCloud2Encoder(BaseEncoder):
    """
    Encodes PointCloud2 with optional decimation.

    For very large point clouds, we can decimate to reduce bandwidth.
    """

    encoding = Encoding.BINARY

    def __init__(self, decimation: int = 1, max_points: int = 50000):
        self.decimation = decimation
        self.max_points = max_points

    def encode(self, msg: Any) -> bytes:
        """Encode point cloud as binary."""
        metadata = self.get_metadata(msg)

        # Get raw point data
        data = np.frombuffer(msg.data, dtype=np.uint8)

        # Apply decimation if needed
        point_step = msg.point_step
        total_points = len(data) // point_step

        if self.decimation > 1 or total_points > self.max_points:
            # Calculate effective decimation
            effective_decimation = max(
                self.decimation,
                total_points // self.max_points + 1
            )

            # Decimate points
            indices = np.arange(0, total_points, effective_decimation)
            decimated_data = []
            for i in indices:
                start = i * point_step
                end = start + point_step
                decimated_data.append(data[start:end])

            data = np.concatenate(decimated_data) if decimated_data else np.array([], dtype=np.uint8)
            metadata['decimated'] = True
            metadata['decimation_factor'] = effective_decimation
            metadata['original_points'] = total_points

        metadata_bytes = cbor2.dumps(metadata)

        return struct.pack('>I', len(metadata_bytes)) + metadata_bytes + data.tobytes()

    def get_metadata(self, msg: Any) -> Dict[str, Any]:
        """Extract point cloud metadata."""
        fields = []
        for field in msg.fields:
            fields.append({
                'name': field.name,
                'offset': field.offset,
                'datatype': field.datatype,
                'count': field.count,
            })

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
            'fields': fields,
            'is_bigendian': msg.is_bigendian,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'is_dense': msg.is_dense,
        }
