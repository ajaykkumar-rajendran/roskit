#!/bin/bash
#
# RosKit Bridge Rust Installation Script
#
# This script builds and installs the roskit_bridge_rs package into your ROS2 workspace.
#
# Usage:
#   ./install.sh                    # Install to default workspace ~/ros2_ws
#   ./install.sh /path/to/ros2_ws   # Install to custom workspace
#   ./install.sh --system           # Install system-wide to /opt/ros/humble
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
RUST_SOURCE_DIR="$(dirname "$PACKAGE_DIR")/roskit-bridge-rs"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

echo_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check for ROS2
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        echo_error "ROS2 environment not sourced. Please run:"
        echo "  source /opt/ros/humble/setup.bash"
        exit 1
    fi
    echo_info "ROS2 $ROS_DISTRO detected"
}

# Check for Rust
check_rust() {
    if ! command -v cargo &> /dev/null; then
        echo_warn "Cargo not found. Installing Rust..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        source "$HOME/.cargo/env"
    fi
    echo_info "Rust $(rustc --version) detected"
}

# Build Rust binary
build_rust() {
    echo_info "Building roskit-bridge (release mode)..."
    cd "$RUST_SOURCE_DIR"
    cargo build --release
    echo_info "Build complete: $RUST_SOURCE_DIR/target/release/roskit-bridge"
}

# Install to ROS2 workspace using colcon
install_workspace() {
    local workspace="$1"

    if [ ! -d "$workspace/src" ]; then
        echo_error "Invalid workspace: $workspace/src not found"
        exit 1
    fi

    echo_info "Installing to workspace: $workspace"

    # Create symlink in workspace src
    local link_path="$workspace/src/roskit_bridge_rs"
    if [ -L "$link_path" ]; then
        rm "$link_path"
    elif [ -d "$link_path" ]; then
        echo_warn "Directory exists at $link_path, skipping symlink"
    else
        ln -sf "$PACKAGE_DIR" "$link_path"
        echo_info "Created symlink: $link_path -> $PACKAGE_DIR"
    fi

    # Build with colcon
    echo_info "Building with colcon..."
    cd "$workspace"
    colcon build --packages-select roskit_bridge_rs --cmake-args -DBUILD_RUST_FROM_SOURCE=OFF

    # Copy pre-built binary
    local bin_dir="$workspace/install/roskit_bridge_rs/lib/roskit_bridge_rs"
    mkdir -p "$bin_dir"
    cp "$RUST_SOURCE_DIR/target/release/roskit-bridge" "$bin_dir/roskit_bridge"
    chmod +x "$bin_dir/roskit_bridge"

    echo_info "Installation complete!"
    echo ""
    echo "To use roskit_bridge_rs:"
    echo "  source $workspace/install/setup.bash"
    echo "  ros2 launch roskit_bridge_rs roskit_bridge.launch.py"
    echo ""
    echo "Or run directly:"
    echo "  ros2 run roskit_bridge_rs roskit_bridge --port 9090"
}

# Install system-wide
install_system() {
    echo_info "Installing system-wide..."

    local install_dir="/opt/ros/$ROS_DISTRO/lib/roskit_bridge_rs"
    local share_dir="/opt/ros/$ROS_DISTRO/share/roskit_bridge_rs"

    sudo mkdir -p "$install_dir"
    sudo mkdir -p "$share_dir/launch"
    sudo mkdir -p "$share_dir/config"

    # Copy binary
    sudo cp "$RUST_SOURCE_DIR/target/release/roskit-bridge" "$install_dir/roskit_bridge"
    sudo chmod +x "$install_dir/roskit_bridge"

    # Copy launch files
    sudo cp "$PACKAGE_DIR/launch/"*.launch.py "$share_dir/launch/"

    # Copy config
    sudo cp "$PACKAGE_DIR/config/"*.yaml "$share_dir/config/"

    # Copy package.xml
    sudo cp "$PACKAGE_DIR/package.xml" "$share_dir/"

    # Create ament index marker
    sudo mkdir -p "/opt/ros/$ROS_DISTRO/share/ament_index/resource_index/packages"
    sudo touch "/opt/ros/$ROS_DISTRO/share/ament_index/resource_index/packages/roskit_bridge_rs"

    echo_info "System-wide installation complete!"
    echo ""
    echo "To use roskit_bridge_rs:"
    echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
    echo "  ros2 launch roskit_bridge_rs roskit_bridge.launch.py"
}

# Main
main() {
    echo "========================================"
    echo "  RosKit Bridge Rust Installer"
    echo "========================================"
    echo ""

    check_ros2
    check_rust
    build_rust

    if [ "$1" == "--system" ]; then
        install_system
    else
        local workspace="${1:-$HOME/ros2_ws}"
        install_workspace "$workspace"
    fi
}

main "$@"
