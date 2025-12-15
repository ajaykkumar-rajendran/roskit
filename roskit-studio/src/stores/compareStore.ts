import { create } from 'zustand'

interface CompareStore {
  selectedRobots: string[]
  isComparing: boolean

  toggleRobotSelection: (robotId: string) => void
  clearSelection: () => void
  startCompare: () => void
  stopCompare: () => void
}

export const useCompareStore = create<CompareStore>((set, get) => ({
  selectedRobots: [],
  isComparing: false,

  toggleRobotSelection: (robotId: string) => {
    const { selectedRobots } = get()
    if (selectedRobots.includes(robotId)) {
      set({ selectedRobots: selectedRobots.filter((id) => id !== robotId) })
    } else {
      // Max 4 robots for comparison
      if (selectedRobots.length < 4) {
        set({ selectedRobots: [...selectedRobots, robotId] })
      }
    }
  },

  clearSelection: () => {
    set({ selectedRobots: [], isComparing: false })
  },

  startCompare: () => {
    if (get().selectedRobots.length >= 2) {
      set({ isComparing: true })
    }
  },

  stopCompare: () => {
    set({ isComparing: false })
  },
}))
