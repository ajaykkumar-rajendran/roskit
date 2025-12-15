export type BTNodeStatus = 'idle' | 'running' | 'success' | 'failure'

export type BTNodeType =
  | 'root'
  | 'sequence'
  | 'selector'
  | 'parallel'
  | 'action'
  | 'condition'
  | 'decorator'
  | 'subtree'

export interface BTNode {
  id: string
  name: string
  type: BTNodeType
  status: BTNodeStatus
  children?: BTNode[]
  // For decorator nodes
  decoratorType?: string
  // For action/condition nodes
  nodeClass?: string
  // Execution metadata
  lastTick?: number
  tickCount?: number
  executionTime?: number
}

export interface BehaviorTree {
  name: string
  rootNode: BTNode
  tickRate?: number
  lastUpdate: number
}

// Status message from /behavior_tree_log topic
export interface BTStatusMessage {
  timestamp: {
    sec: number
    nanosec: number
  }
  node_name: string
  node_status: string
  previous_status?: string
}

// Tree structure from /behavior_tree_xml or service
export interface BTTreeMessage {
  tree_xml?: string
  tree_name?: string
}
