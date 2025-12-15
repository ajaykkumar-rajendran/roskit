import { useEffect, useRef, useCallback } from 'react'
import * as THREE from 'three'
import { useMapStore } from '@/stores/mapStore'

interface UseThreeSceneResult {
  containerRef: React.RefObject<HTMLDivElement | null>
  sceneRef: React.RefObject<THREE.Scene | null>
  cameraRef: React.RefObject<THREE.OrthographicCamera | null>
  rendererRef: React.RefObject<THREE.WebGLRenderer | null>
  zoomIn: () => void
  zoomOut: () => void
  resetView: () => void
}

const MIN_ZOOM = 0.1
const MAX_ZOOM = 50
const ZOOM_STEP = 1.2

export function useThreeScene(): UseThreeSceneResult {
  const containerRef = useRef<HTMLDivElement>(null)
  const sceneRef = useRef<THREE.Scene | null>(null)
  const cameraRef = useRef<THREE.OrthographicCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const animationFrameRef = useRef<number | null>(null)
  const isDraggingRef = useRef(false)
  const lastMousePosRef = useRef({ x: 0, y: 0 })

  const { viewState, setZoom, setCenter, setFollowRobot } = useMapStore()

  const updateCameraFromState = useCallback(() => {
    if (!cameraRef.current) return

    const camera = cameraRef.current
    const zoom = viewState.zoom

    // Update camera zoom (frustum size)
    const aspect = camera.right / camera.top
    const frustumSize = 20 / zoom
    camera.left = (-frustumSize * aspect) / 2
    camera.right = (frustumSize * aspect) / 2
    camera.top = frustumSize / 2
    camera.bottom = -frustumSize / 2

    // Update camera position
    camera.position.x = viewState.centerX
    camera.position.y = viewState.centerY

    camera.updateProjectionMatrix()
  }, [viewState])

  // Initialize Three.js scene
  useEffect(() => {
    if (!containerRef.current) return

    const container = containerRef.current
    const width = container.clientWidth
    const height = container.clientHeight

    // Create scene
    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x1a1a2e)
    sceneRef.current = scene

    // Create orthographic camera for 2D top-down view
    const aspect = width / height
    const frustumSize = 20
    const camera = new THREE.OrthographicCamera(
      (-frustumSize * aspect) / 2,
      (frustumSize * aspect) / 2,
      frustumSize / 2,
      -frustumSize / 2,
      0.1,
      1000
    )
    camera.position.set(0, 0, 100)
    camera.lookAt(0, 0, 0)
    cameraRef.current = camera

    // Create renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setSize(width, height)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    container.appendChild(renderer.domElement)
    rendererRef.current = renderer

    // Add grid helper
    const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x333333)
    gridHelper.rotation.x = Math.PI / 2 // Rotate to XY plane
    gridHelper.position.z = -0.1 // Slightly below other elements
    scene.add(gridHelper)

    // Add axes helper (small, at origin)
    const axesHelper = new THREE.AxesHelper(2)
    axesHelper.position.z = 0.1
    scene.add(axesHelper)

    // Animation loop
    const animate = () => {
      animationFrameRef.current = requestAnimationFrame(animate)
      renderer.render(scene, camera)
    }
    animate()

    // Handle resize
    const handleResize = () => {
      if (!containerRef.current || !cameraRef.current || !rendererRef.current) return

      const newWidth = containerRef.current.clientWidth
      const newHeight = containerRef.current.clientHeight
      const newAspect = newWidth / newHeight

      const currentFrustumSize = cameraRef.current.top - cameraRef.current.bottom
      cameraRef.current.left = (-currentFrustumSize * newAspect) / 2
      cameraRef.current.right = (currentFrustumSize * newAspect) / 2
      cameraRef.current.updateProjectionMatrix()

      rendererRef.current.setSize(newWidth, newHeight)
    }

    window.addEventListener('resize', handleResize)

    // Handle mouse events for pan
    const handleMouseDown = (event: MouseEvent) => {
      if (event.button === 0) { // Left click
        isDraggingRef.current = true
        lastMousePosRef.current = { x: event.clientX, y: event.clientY }
        setFollowRobot(false) // Disable follow robot when user pans
      }
    }

    const handleMouseMove = (event: MouseEvent) => {
      if (!isDraggingRef.current || !cameraRef.current) return

      const deltaX = event.clientX - lastMousePosRef.current.x
      const deltaY = event.clientY - lastMousePosRef.current.y

      // Calculate pan amount based on camera frustum size
      const frustumWidth = cameraRef.current.right - cameraRef.current.left
      const frustumHeight = cameraRef.current.top - cameraRef.current.bottom
      const containerWidth = containerRef.current?.clientWidth || 1
      const containerHeight = containerRef.current?.clientHeight || 1

      const panX = -(deltaX / containerWidth) * frustumWidth
      const panY = (deltaY / containerHeight) * frustumHeight

      setCenter(viewState.centerX + panX, viewState.centerY + panY)

      lastMousePosRef.current = { x: event.clientX, y: event.clientY }
    }

    const handleMouseUp = () => {
      isDraggingRef.current = false
    }

    const handleWheel = (event: WheelEvent) => {
      event.preventDefault()
      const zoomFactor = event.deltaY > 0 ? 1 / ZOOM_STEP : ZOOM_STEP
      const newZoom = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, viewState.zoom * zoomFactor))
      setZoom(newZoom)
      setFollowRobot(false) // Disable follow robot when user zooms
    }

    renderer.domElement.addEventListener('mousedown', handleMouseDown)
    renderer.domElement.addEventListener('mousemove', handleMouseMove)
    renderer.domElement.addEventListener('mouseup', handleMouseUp)
    renderer.domElement.addEventListener('mouseleave', handleMouseUp)
    renderer.domElement.addEventListener('wheel', handleWheel, { passive: false })

    // Cleanup
    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current)
      }

      renderer.domElement.removeEventListener('mousedown', handleMouseDown)
      renderer.domElement.removeEventListener('mousemove', handleMouseMove)
      renderer.domElement.removeEventListener('mouseup', handleMouseUp)
      renderer.domElement.removeEventListener('mouseleave', handleMouseUp)
      renderer.domElement.removeEventListener('wheel', handleWheel)
      window.removeEventListener('resize', handleResize)

      // Dispose Three.js objects
      scene.traverse((object) => {
        if (object instanceof THREE.Mesh) {
          object.geometry.dispose()
          if (Array.isArray(object.material)) {
            object.material.forEach((m) => m.dispose())
          } else {
            object.material.dispose()
          }
        }
      })

      renderer.dispose()
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement)
      }

      sceneRef.current = null
      cameraRef.current = null
      rendererRef.current = null
    }
  }, [])

  // Update camera when view state changes
  useEffect(() => {
    updateCameraFromState()
  }, [updateCameraFromState])

  const zoomIn = useCallback(() => {
    const newZoom = Math.min(MAX_ZOOM, viewState.zoom * ZOOM_STEP)
    setZoom(newZoom)
  }, [viewState.zoom, setZoom])

  const zoomOut = useCallback(() => {
    const newZoom = Math.max(MIN_ZOOM, viewState.zoom / ZOOM_STEP)
    setZoom(newZoom)
  }, [viewState.zoom, setZoom])

  const resetViewHandler = useCallback(() => {
    setZoom(1)
    setCenter(0, 0)
    setFollowRobot(true)
  }, [setZoom, setCenter, setFollowRobot])

  return {
    containerRef,
    sceneRef,
    cameraRef,
    rendererRef,
    zoomIn,
    zoomOut,
    resetView: resetViewHandler,
  }
}
