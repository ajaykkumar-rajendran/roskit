import { create } from 'zustand'
import type { RosKitClient, Subscription as RosKitSubscription } from '@roskit/client'
import type { BTNode, BTNodeStatus, BTNodeType, BehaviorTree } from '@/types/behaviortree'

interface BehaviorTreeStore {
  trees: Record<string, BehaviorTree>
  activeTreeId: string | null
  isSubscribed: boolean
  statusSubscription: RosKitSubscription | null

  setActiveTree: (treeId: string | null) => void
  subscribeToStatus: (client: RosKitClient, robotId: string) => Promise<void>
  unsubscribeFromStatus: () => void
  updateNodeStatus: (robotId: string, nodeName: string, status: BTNodeStatus) => void
  loadTreeFromXML: (robotId: string, xml: string, treeName: string) => void
  loadDemoTree: (robotId: string) => void
}

// Parse BehaviorTree XML to our node structure
function parseXMLToTree(xml: string): BTNode | null {
  try {
    const parser = new DOMParser()
    const doc = parser.parseFromString(xml, 'text/xml')

    const rootEl = doc.querySelector('BehaviorTree > *') || doc.querySelector('root > *')
    if (!rootEl) return null

    return parseNode(rootEl, '0')
  } catch {
    return null
  }
}

function parseNode(element: Element, idPrefix: string): BTNode {
  const tagName = element.tagName.toLowerCase()
  const name = element.getAttribute('name') || element.getAttribute('ID') || tagName

  const type = mapTagToType(tagName)
  const children: BTNode[] = []

  Array.from(element.children).forEach((child, index) => {
    children.push(parseNode(child, `${idPrefix}-${index}`))
  })

  return {
    id: idPrefix,
    name,
    type,
    status: 'idle',
    children: children.length > 0 ? children : undefined,
    nodeClass: element.getAttribute('class') || undefined,
    decoratorType: type === 'decorator' ? tagName : undefined,
  }
}

function mapTagToType(tagName: string): BTNodeType {
  const tag = tagName.toLowerCase()

  if (tag === 'sequence' || tag === 'reactivesequence') return 'sequence'
  if (tag === 'fallback' || tag === 'selector' || tag === 'reactivefallback') return 'selector'
  if (tag === 'parallel') return 'parallel'
  if (tag === 'subtree' || tag === 'subtreenode') return 'subtree'
  if (tag === 'condition' || tag.includes('condition')) return 'condition'
  if (tag === 'root' || tag === 'behaviortree') return 'root'

  // Decorators
  const decorators = ['inverter', 'repeat', 'retry', 'timeout', 'delay', 'forcesuccess', 'forcefailure', 'keeprunning']
  if (decorators.includes(tag)) return 'decorator'

  // Default to action
  return 'action'
}

// Create a demo tree for testing
function createDemoTree(): BTNode {
  return {
    id: 'root',
    name: 'Root',
    type: 'root',
    status: 'idle',
    children: [
      {
        id: 'seq-main',
        name: 'MainSequence',
        type: 'sequence',
        status: 'idle',
        children: [
          {
            id: 'cond-battery',
            name: 'BatteryOK',
            type: 'condition',
            status: 'success',
            nodeClass: 'BatteryCondition',
          },
          {
            id: 'sel-nav',
            name: 'NavigationSelector',
            type: 'selector',
            status: 'idle',
            children: [
              {
                id: 'cond-at-goal',
                name: 'AtGoal',
                type: 'condition',
                status: 'failure',
                nodeClass: 'GoalReachedCondition',
              },
              {
                id: 'seq-navigate',
                name: 'NavigateSequence',
                type: 'sequence',
                status: 'running',
                children: [
                  {
                    id: 'action-compute',
                    name: 'ComputePath',
                    type: 'action',
                    status: 'success',
                    nodeClass: 'ComputePathToPose',
                  },
                  {
                    id: 'action-follow',
                    name: 'FollowPath',
                    type: 'action',
                    status: 'running',
                    nodeClass: 'FollowPath',
                    executionTime: 1250,
                  },
                ],
              },
            ],
          },
          {
            id: 'dec-retry',
            name: 'RetryClean',
            type: 'decorator',
            status: 'idle',
            decoratorType: 'Retry',
            children: [
              {
                id: 'action-clean',
                name: 'PerformCleaning',
                type: 'action',
                status: 'idle',
                nodeClass: 'CleaningAction',
              },
            ],
          },
        ],
      },
    ],
  }
}

export const useBehaviorTreeStore = create<BehaviorTreeStore>((set, get) => ({
  trees: {},
  activeTreeId: null,
  isSubscribed: false,
  statusSubscription: null,

  setActiveTree: (treeId) => {
    set({ activeTreeId: treeId })
  },

  subscribeToStatus: async (client, robotId) => {
    const { statusSubscription } = get()
    if (statusSubscription) {
      statusSubscription.unsubscribe()
    }

    try {
      // Subscribe to BT status topic (common topics used by BehaviorTree.CPP / Nav2)
      const sub = await client.subscribe<{
        status_changes?: Array<{ node_name: string; current_status: string }>
      }>(
        '/behavior_tree_log',
        (message) => {
          if (message.status_changes) {
            message.status_changes.forEach((change) => {
              const status = mapStatusString(change.current_status)
              get().updateNodeStatus(robotId, change.node_name, status)
            })
          }
        },
        { msgType: 'behavior_tree_msgs/msg/StatusChangeLog' }
      )

      set({ statusSubscription: sub, isSubscribed: true })
    } catch (error) {
      console.error('Failed to subscribe to behavior tree status:', error)
    }
  },

  unsubscribeFromStatus: () => {
    const { statusSubscription } = get()
    if (statusSubscription) {
      statusSubscription.unsubscribe()
    }
    set({ statusSubscription: null, isSubscribed: false })
  },

  updateNodeStatus: (robotId, nodeName, status) => {
    set((state) => {
      const tree = state.trees[robotId]
      if (!tree) return state

      const updatedRoot = updateNodeStatusRecursive(tree.rootNode, nodeName, status)

      return {
        trees: {
          ...state.trees,
          [robotId]: {
            ...tree,
            rootNode: updatedRoot,
            lastUpdate: Date.now(),
          },
        },
      }
    })
  },

  loadTreeFromXML: (robotId, xml, treeName) => {
    const rootNode = parseXMLToTree(xml)
    if (!rootNode) return

    set((state) => ({
      trees: {
        ...state.trees,
        [robotId]: {
          name: treeName,
          rootNode,
          lastUpdate: Date.now(),
        },
      },
      activeTreeId: robotId,
    }))
  },

  loadDemoTree: (robotId) => {
    set((state) => ({
      trees: {
        ...state.trees,
        [robotId]: {
          name: 'CleaningRobotBT',
          rootNode: createDemoTree(),
          lastUpdate: Date.now(),
        },
      },
      activeTreeId: robotId,
    }))
  },
}))

function updateNodeStatusRecursive(node: BTNode, nodeName: string, status: BTNodeStatus): BTNode {
  if (node.name === nodeName) {
    return { ...node, status, lastTick: Date.now() }
  }

  if (node.children) {
    return {
      ...node,
      children: node.children.map((child) =>
        updateNodeStatusRecursive(child, nodeName, status)
      ),
    }
  }

  return node
}

function mapStatusString(statusStr: string): BTNodeStatus {
  const s = statusStr.toLowerCase()
  if (s === 'running' || s === '1') return 'running'
  if (s === 'success' || s === '2') return 'success'
  if (s === 'failure' || s === '3') return 'failure'
  return 'idle'
}
