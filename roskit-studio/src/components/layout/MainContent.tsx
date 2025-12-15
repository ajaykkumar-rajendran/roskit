import { useState } from 'react'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Radio, Box, GitBranch, Activity, Eye, Wrench, TreeDeciduous, LayoutDashboard, Settings2, Map } from 'lucide-react'
import { TopicList } from '@/components/topics/TopicList'
import { TopicSubscriber } from '@/components/topics/TopicSubscriber'
import { NodeList } from '@/components/nodes/NodeList'
import { GraphView } from '@/components/graph/GraphView'
import { BenchmarkPanel } from '@/components/benchmark/BenchmarkPanel'
import { CompareView } from '@/components/compare/CompareView'
import { ServiceList } from '@/components/services/ServiceList'
import { BehaviorTreeView } from '@/components/behaviortree/BehaviorTreeView'
import { DashboardView } from '@/components/dashboard/DashboardView'
import { ParameterEditor } from '@/components/parameters/ParameterEditor'
import { MapView } from '@/components/map/MapView'
import { useTopicStore } from '@/stores/topicStore'
import { useConnectionStore } from '@/stores/connectionStore'
import { useCompareStore } from '@/stores/compareStore'
import { ScrollArea } from '@/components/ui/scroll-area'

export function MainContent() {
  const [activeTab, setActiveTab] = useState('topics')
  const subscriptions = useTopicStore((s) => s.subscriptions)
  const activeRobotId = useConnectionStore((s) => s.activeRobotId)
  const isComparing = useCompareStore((s) => s.isComparing)

  const activeSubscriptions = Object.entries(subscriptions).filter(
    ([key, sub]) =>
      activeRobotId && key.startsWith(`${activeRobotId}:`) && sub.isSubscribed
  )

  // Show compare view when in comparison mode
  if (isComparing) {
    return <CompareView />
  }

  return (
    <div className="flex-1 flex overflow-hidden">
      {/* Main Panel */}
      <div className="flex-1 flex flex-col min-w-0 min-h-0">
        <Tabs value={activeTab} onValueChange={setActiveTab} className="flex-1 flex flex-col min-h-0">
          <div className="border-b border-[hsl(var(--border))] px-4 shrink-0">
            <TabsList className="h-12 bg-transparent">
              <TabsTrigger value="topics" className="gap-2">
                <Radio className="h-4 w-4" />
                Topics
              </TabsTrigger>
              <TabsTrigger value="nodes" className="gap-2">
                <Box className="h-4 w-4" />
                Nodes
              </TabsTrigger>
              <TabsTrigger value="graph" className="gap-2">
                <GitBranch className="h-4 w-4" />
                Graph
              </TabsTrigger>
              <TabsTrigger value="benchmark" className="gap-2">
                <Activity className="h-4 w-4" />
                Benchmark
              </TabsTrigger>
              <TabsTrigger value="services" className="gap-2">
                <Wrench className="h-4 w-4" />
                Services
              </TabsTrigger>
              <TabsTrigger value="behaviortree" className="gap-2">
                <TreeDeciduous className="h-4 w-4" />
                BT
              </TabsTrigger>
              <TabsTrigger value="dashboard" className="gap-2">
                <LayoutDashboard className="h-4 w-4" />
                Dashboard
              </TabsTrigger>
              <TabsTrigger value="parameters" className="gap-2">
                <Settings2 className="h-4 w-4" />
                Params
              </TabsTrigger>
              <TabsTrigger value="map" className="gap-2">
                <Map className="h-4 w-4" />
                Map
              </TabsTrigger>
            </TabsList>
          </div>

          <TabsContent value="topics" className="flex-1 m-0 min-h-0 overflow-hidden">
            <TopicList />
          </TabsContent>

          <TabsContent value="nodes" className="flex-1 m-0 min-h-0 overflow-hidden">
            <NodeList />
          </TabsContent>

          <TabsContent value="graph" className="flex-1 m-0 min-h-0 overflow-hidden">
            <GraphView />
          </TabsContent>

          <TabsContent value="benchmark" className="flex-1 m-0 min-h-0 overflow-hidden">
            <BenchmarkPanel />
          </TabsContent>

          <TabsContent value="services" className="flex-1 m-0 min-h-0 overflow-hidden">
            <ServiceList />
          </TabsContent>

          <TabsContent value="behaviortree" className="flex-1 m-0 min-h-0 overflow-hidden">
            <BehaviorTreeView />
          </TabsContent>

          <TabsContent value="dashboard" className="flex-1 m-0 min-h-0 overflow-hidden">
            <DashboardView />
          </TabsContent>

          <TabsContent value="parameters" className="flex-1 m-0 min-h-0 overflow-hidden">
            <ParameterEditor />
          </TabsContent>

          <TabsContent value="map" className="flex-1 m-0 min-h-0 overflow-hidden">
            <MapView />
          </TabsContent>
        </Tabs>
      </div>

      {/* Subscribed Topics Panel */}
      {activeSubscriptions.length > 0 && (
        <div className="w-[400px] border-l border-[hsl(var(--border))] flex flex-col">
          <div className="p-4 border-b border-[hsl(var(--border))] flex items-center gap-2">
            <Eye className="h-4 w-4" />
            <h3 className="font-semibold">Live Data</h3>
            <span className="text-sm text-[hsl(var(--muted-foreground))]">
              ({activeSubscriptions.length})
            </span>
          </div>
          <ScrollArea className="flex-1">
            <div className="p-2 space-y-2">
              {activeSubscriptions.map(([key]) => (
                <div key={key} className="h-[300px]">
                  <TopicSubscriber
                    subscriptionKey={key}
                    onClose={() => {}}
                  />
                </div>
              ))}
            </div>
          </ScrollArea>
        </div>
      )}
    </div>
  )
}
