import type { TopicInfo } from '@roskit/client'
import type { RosKitClient } from '@roskit/client'

export async function getRoskitTopics(client: RosKitClient): Promise<TopicInfo[]> {
  return client.getTopics()
}
