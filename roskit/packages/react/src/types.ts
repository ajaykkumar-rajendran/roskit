/**
 * RosKit React Types
 *
 * Status enums and retry policy types for React hooks.
 */

/**
 * Subscription status for useSubscription and derived hooks
 */
export enum SubscriptionStatus {
  /** Not subscribed, not attempting to subscribe */
  IDLE = 'idle',
  /** Attempting to subscribe */
  SUBSCRIBING = 'subscribing',
  /** Successfully subscribed and receiving messages */
  SUBSCRIBED = 'subscribed',
  /** Subscription failed, may be retrying */
  ERROR = 'error',
  /** Retrying subscription after failure */
  RETRYING = 'retrying',
  /** Disabled by user (enabled=false) */
  DISABLED = 'disabled',
  /** Waiting for connection before subscribing */
  WAITING_FOR_CONNECTION = 'waiting_for_connection',
}

/**
 * Service call status for useService hook
 */
export enum ServiceCallStatus {
  /** No call in progress */
  IDLE = 'idle',
  /** Call in progress */
  CALLING = 'calling',
  /** Call succeeded */
  SUCCESS = 'success',
  /** Call failed */
  ERROR = 'error',
}

/**
 * Connection status (extends ConnectionState with reconnecting)
 */
export enum ConnectionStatus {
  /** Not connected */
  DISCONNECTED = 'disconnected',
  /** Attempting to connect */
  CONNECTING = 'connecting',
  /** Connected and ready */
  CONNECTED = 'connected',
  /** Connection error */
  ERROR = 'error',
  /** Reconnecting after disconnect */
  RECONNECTING = 'reconnecting',
}

/**
 * Retry policy configuration
 */
export interface RetryPolicy {
  /** Maximum number of retry attempts (0 = no retries, -1 = unlimited) */
  maxRetries?: number
  /** Initial delay between retries in ms */
  initialDelayMs?: number
  /** Maximum delay between retries in ms */
  maxDelayMs?: number
  /** Backoff multiplier (e.g., 2 for exponential backoff) */
  backoffMultiplier?: number
  /** Add random jitter to delays (0-1, percentage of delay) */
  jitter?: number
  /** Retry only on specific error kinds */
  retryOn?: Array<'network' | 'timeout' | 'server' | 'all'>
}

/**
 * Default retry policy for subscriptions
 */
export const DEFAULT_SUBSCRIPTION_RETRY_POLICY: Required<RetryPolicy> = {
  maxRetries: 3,
  initialDelayMs: 300,
  maxDelayMs: 5000,
  backoffMultiplier: 2,
  jitter: 0.1,
  retryOn: ['network', 'timeout'],
}

/**
 * Default retry policy for service calls
 */
export const DEFAULT_SERVICE_RETRY_POLICY: Required<RetryPolicy> = {
  maxRetries: 0, // Service calls don't retry by default
  initialDelayMs: 500,
  maxDelayMs: 5000,
  backoffMultiplier: 2,
  jitter: 0.1,
  retryOn: ['network', 'timeout'],
}

/**
 * Calculate retry delay with exponential backoff and jitter
 */
export function calculateRetryDelay(
  attempt: number,
  policy: Required<RetryPolicy>
): number {
  const baseDelay = Math.min(
    policy.initialDelayMs * Math.pow(policy.backoffMultiplier, attempt),
    policy.maxDelayMs
  )

  // Add jitter
  const jitterAmount = baseDelay * policy.jitter * (Math.random() * 2 - 1)
  return Math.max(0, Math.round(baseDelay + jitterAmount))
}

/**
 * Check if an error should trigger a retry
 */
export function shouldRetry(
  error: Error,
  policy: Required<RetryPolicy>,
  attempt: number
): boolean {
  // Check max retries
  if (policy.maxRetries !== -1 && attempt >= policy.maxRetries) {
    return false
  }

  // Check error kind
  if (policy.retryOn.includes('all')) {
    return true
  }

  // Check for RosKitError kind
  const errorKind = (error as any)?.kind
  if (errorKind && policy.retryOn.includes(errorKind)) {
    return true
  }

  // Fallback: retry on network-like errors
  const message = error.message.toLowerCase()
  if (
    policy.retryOn.includes('network') &&
    (message.includes('network') ||
      message.includes('connection') ||
      message.includes('websocket'))
  ) {
    return true
  }

  if (
    policy.retryOn.includes('timeout') &&
    (message.includes('timeout') || message.includes('timed out'))
  ) {
    return true
  }

  return false
}

/**
 * Retry state for hooks
 */
export interface RetryState {
  /** Current retry attempt (0 = first try) */
  attempt: number
  /** Next retry scheduled at (timestamp) */
  nextRetryAt: number | null
  /** Time until next retry in ms */
  retryInMs: number | null
}
