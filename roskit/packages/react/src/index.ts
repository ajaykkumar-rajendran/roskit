/**
 * @roskit/react
 *
 * React hooks and components for RosKit Bridge.
 */

// Context and providers
export {
  RosKitProvider,
  useRosKit,
  useRosKitClient,
  // Legacy exports for backward compatibility
  RosKitProvider as RosWebProvider,
  useRosKit as useRosWeb,
  useRosKitClient as useRosWebClient,
} from './context'

// Hooks
export {
  useSubscription,
  useJsonSubscription,
  useLaserScan,
  useOccupancyGrid,
  usePose,
  usePath,
  useTopics,
  useService,
} from './hooks'

// Types and enums
export {
  SubscriptionStatus,
  ServiceCallStatus,
  ConnectionStatus,
  DEFAULT_SUBSCRIPTION_RETRY_POLICY,
  DEFAULT_SERVICE_RETRY_POLICY,
  calculateRetryDelay,
  shouldRetry,
} from './types'

export type { RetryPolicy, RetryState } from './types'

// Re-export common types from client
export type {
  ConnectionState,
  RosKitClientConfig,
  RosWebClientConfig,
  SubscriptionOptions,
  Subscription,
  MessageCallback,
  ServerInfo,
  TopicInfo,
  Header,
  Point,
  Quaternion,
  Pose,
  PoseStamped,
  MapMetaData,
  OccupancyGrid,
  LaserScan,
  Path,
  Twist,
  TwistStamped,
} from '@roskit/client'
