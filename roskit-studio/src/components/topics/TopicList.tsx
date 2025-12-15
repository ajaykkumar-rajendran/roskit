import { useState, useMemo, useRef, useEffect } from 'react'
import { Search, Radio, Filter, SortAsc, ChevronDown, ChevronUp } from 'lucide-react'
import { Input } from '@/components/ui/input'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { TopicRow } from './TopicRow'
import { useConnectionStore } from '@/stores/connectionStore'
import { useTopicStore } from '@/stores/topicStore'

type SortOption = 'name' | 'type' | 'subscribed'

const INITIAL_VISIBLE_TYPES = 10
const ITEMS_PER_PAGE = 30

export function TopicList() {
  const [search, setSearch] = useState('')
  const [selectedType, setSelectedType] = useState<string | null>(null)
  const [sortBy, setSortBy] = useState<SortOption>('name')
  const [showAllTypes, setShowAllTypes] = useState(false)
  const [visibleCount, setVisibleCount] = useState(ITEMS_PER_PAGE)
  const { robots, activeRobotId } = useConnectionStore()
  const subscriptions = useTopicStore((s) => s.subscriptions)
  const loadMoreRef = useRef<HTMLDivElement>(null)

  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  // Get unique message types for filtering
  const messageTypes = useMemo(() => {
    if (!activeRobot) return []
    const types = new Set(activeRobot.topics.map((t) => t.type))
    return Array.from(types).sort()
  }, [activeRobot])

  const filteredTopics = useMemo(() => {
    if (!activeRobot) return []

    const searchLower = search.toLowerCase()
    let filtered = activeRobot.topics.filter(
      (topic) =>
        (topic.name.toLowerCase().includes(searchLower) ||
          topic.type.toLowerCase().includes(searchLower)) &&
        (!selectedType || topic.type === selectedType)
    )

    // Sort topics
    filtered = [...filtered].sort((a, b) => {
      switch (sortBy) {
        case 'type':
          return a.type.localeCompare(b.type)
        case 'subscribed': {
          const aSubscribed = subscriptions[`${activeRobot.id}:${a.name}`]?.isSubscribed
          const bSubscribed = subscriptions[`${activeRobot.id}:${b.name}`]?.isSubscribed
          if (aSubscribed && !bSubscribed) return -1
          if (!aSubscribed && bSubscribed) return 1
          return a.name.localeCompare(b.name)
        }
        default:
          return a.name.localeCompare(b.name)
      }
    })

    return filtered
  }, [activeRobot, search, selectedType, sortBy, subscriptions])

  // Visible topics for infinite scroll
  const visibleTopics = useMemo(() => {
    return filteredTopics.slice(0, visibleCount)
  }, [filteredTopics, visibleCount])

  const hasMore = visibleCount < filteredTopics.length

  // Reset visible count when filters change
  useEffect(() => {
    setVisibleCount(ITEMS_PER_PAGE)
  }, [search, selectedType, sortBy, activeRobotId])

  // Infinite scroll observer
  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        if (entries[0].isIntersecting && hasMore) {
          setVisibleCount((prev) => Math.min(prev + ITEMS_PER_PAGE, filteredTopics.length))
        }
      },
      { threshold: 0.1 }
    )

    const currentRef = loadMoreRef.current
    if (currentRef) {
      observer.observe(currentRef)
    }

    return () => {
      if (currentRef) {
        observer.unobserve(currentRef)
      }
    }
  }, [hasMore, filteredTopics.length])

  // Types to display based on expanded state
  const displayedTypes = showAllTypes
    ? messageTypes
    : messageTypes.slice(0, INITIAL_VISIBLE_TYPES)
  const hiddenTypesCount = messageTypes.length - INITIAL_VISIBLE_TYPES

  if (!activeRobot) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Radio className="h-12 w-12 opacity-50" />
        <p className="mt-3">Select a robot to view topics</p>
      </div>
    )
  }

  if (activeRobot.status !== 'connected') {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Radio className="h-12 w-12 opacity-50" />
        <p className="mt-3">Robot is not connected</p>
      </div>
    )
  }

  return (
    <div className="flex flex-col h-full overflow-hidden">
      <div className="p-4 border-b border-[hsl(var(--border))] space-y-3 shrink-0">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <h2 className="font-semibold">Topics</h2>
            <span className="text-sm text-[hsl(var(--muted-foreground))]">
              ({filteredTopics.length})
            </span>
          </div>
          <div className="flex items-center gap-2">
            <Button
              variant={sortBy === 'name' ? 'secondary' : 'ghost'}
              size="sm"
              onClick={() => setSortBy('name')}
            >
              <SortAsc className="h-3.5 w-3.5 mr-1" />
              Name
            </Button>
            <Button
              variant={sortBy === 'type' ? 'secondary' : 'ghost'}
              size="sm"
              onClick={() => setSortBy('type')}
            >
              <Filter className="h-3.5 w-3.5 mr-1" />
              Type
            </Button>
            <Button
              variant={sortBy === 'subscribed' ? 'secondary' : 'ghost'}
              size="sm"
              onClick={() => setSortBy('subscribed')}
            >
              Subscribed
            </Button>
          </div>
        </div>
        <div className="relative">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-[hsl(var(--muted-foreground))]" />
          <Input
            placeholder="Search topics by name or type..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="pl-9"
          />
        </div>
        {messageTypes.length > 1 && (
          <div className="flex flex-wrap gap-1">
            <Badge
              variant={selectedType === null ? 'default' : 'outline'}
              className="cursor-pointer"
              onClick={() => setSelectedType(null)}
            >
              All
            </Badge>
            {displayedTypes.map((type) => (
              <Badge
                key={type}
                variant={selectedType === type ? 'default' : 'outline'}
                className="cursor-pointer font-mono text-xs"
                onClick={() => setSelectedType(selectedType === type ? null : type)}
              >
                {type.split('/').pop()}
              </Badge>
            ))}
            {hiddenTypesCount > 0 && (
              <Badge
                variant="outline"
                className="cursor-pointer text-xs gap-1 hover:bg-[hsl(var(--muted))]"
                onClick={() => setShowAllTypes(!showAllTypes)}
              >
                {showAllTypes ? (
                  <>
                    <ChevronUp className="h-3 w-3" />
                    Show less
                  </>
                ) : (
                  <>
                    <ChevronDown className="h-3 w-3" />
                    +{hiddenTypesCount} more
                  </>
                )}
              </Badge>
            )}
          </div>
        )}
      </div>

      <div className="flex-1 min-h-0 overflow-y-auto">
        <div className="divide-y divide-[hsl(var(--border))]">
          {filteredTopics.length === 0 ? (
            <div className="p-4 text-center text-[hsl(var(--muted-foreground))]">
              {search ? 'No topics match your search' : 'No topics available'}
            </div>
          ) : (
            <>
              {visibleTopics.map((topic) => (
                <TopicRow
                  key={topic.name}
                  topic={topic}
                  robotId={activeRobot.id}
                />
              ))}
              {/* Infinite scroll trigger */}
              {hasMore && (
                <div
                  ref={loadMoreRef}
                  className="p-4 text-center text-sm text-[hsl(var(--muted-foreground))]"
                >
                  Loading more topics...
                </div>
              )}
              {!hasMore && filteredTopics.length > ITEMS_PER_PAGE && (
                <div className="p-3 text-center text-xs text-[hsl(var(--muted-foreground))]">
                  Showing all {filteredTopics.length} topics
                </div>
              )}
            </>
          )}
        </div>
      </div>
    </div>
  )
}
