# RosWeb

High-performance web bridge and SDK for ROS2.

## Packages

| Package | Description | Status |
|---------|-------------|--------|
| [rosweb_bridge](./bridge/rosweb_bridge) | ROS2 Python node with binary WebSocket server | 🚧 Development |
| [@rosweb/client](./packages/client) | TypeScript client SDK | 🚧 Development |
| [@rosweb/react](./packages/react) | React hooks and components | 🚧 Development |

## Why RosWeb?

- **10-100x faster** than rosbridge for large messages (maps, LiDAR, images)
- **Binary protocol** with smart encoding per message type
- **Developer-friendly** TypeScript SDK with React hooks
- **Easy deployment** - single ROS2 package
- **Fully open source** - MIT licensed

## Quick Start

### On the Robot

```bash
# Install the bridge
cd bridge/rosweb_bridge
colcon build --packages-select rosweb_bridge
source install/setup.bash

# Run the bridge
ros2 run rosweb_bridge bridge --ros-args -p port:=9091
```

### In Your Web App

```bash
npm install @rosweb/client @rosweb/react
```

```tsx
import { RosWebProvider, useLaserScan, useOccupancyGrid, usePose } from '@rosweb/react';

function App() {
  return (
    <RosWebProvider config={{ url: 'ws://robot:9091' }} autoConnect>
      <MapDisplay />
    </RosWebProvider>
  );
}

function MapDisplay() {
  const { points } = useLaserScan('/scan');
  const { imageData, info } = useOccupancyGrid('/map');
  const { position, yawDegrees } = usePose('/robot_pose');

  return (
    <div>
      <p>LiDAR Points: {points.length}</p>
      <p>Map: {info?.width}x{info?.height}</p>
      <p>Robot: ({position?.x.toFixed(2)}, {position?.y.toFixed(2)}) @ {yawDegrees?.toFixed(1)}°</p>
    </div>
  );
}
```

## Development

```bash
# Install dependencies
npm install

# Build all packages
npm run build

# Build specific package
npm run build:client
npm run build:react

# Development mode (watch)
npm run dev
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Web Application                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  @rosweb/react (hooks, components)                   │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  @rosweb/client (protocol, decoders)                 │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                              │
                    WebSocket (Binary)
                              │
┌─────────────────────────────────────────────────────────────┐
│  Robot                                                       │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  rosweb_bridge (ROS2 node)                           │    │
│  │  - PNG encoding for maps                             │    │
│  │  - Binary arrays for LiDAR                           │    │
│  │  - CBOR for structured data                          │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ROS2 (DDS)                                          │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## API Reference

### @rosweb/client

Core client for connecting to RosWeb Bridge:

```ts
import { RosWebClient } from '@rosweb/client';

const client = new RosWebClient({ url: 'ws://localhost:9091' });

// Connect
await client.connect();

// Subscribe to topics
const subscription = await client.subscribe('/scan', (message, timestamp) => {
  console.log('Received:', message);
});

// Get available topics
const topics = await client.getTopics();

// Disconnect
client.disconnect();
```

### @rosweb/react

React hooks for ROS2 data:

| Hook | Description |
|------|-------------|
| `useRosWeb()` | Access connection state and client |
| `useSubscription(topic)` | Subscribe to any topic |
| `useLaserScan(topic)` | LaserScan with Cartesian points |
| `useOccupancyGrid(topic)` | Map with RGBA image data |
| `usePose(topic)` | Pose with computed yaw angle |
| `usePath(topic)` | Path with length calculation |
| `useTopics()` | List available topics |

### Binary Protocol

RosWeb uses a custom binary protocol for optimal performance:

| Encoding | Used For | Benefit |
|----------|----------|---------|
| PNG | OccupancyGrid | 10x smaller than JSON |
| Binary Float32 | LaserScan, PointCloud | Zero parsing overhead |
| CBOR | Structured messages | 2-3x smaller than JSON |
| JPEG | Camera images | Native compression |

## License

MIT
