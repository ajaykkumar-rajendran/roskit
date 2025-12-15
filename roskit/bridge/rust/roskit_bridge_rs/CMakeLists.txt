cmake_minimum_required(VERSION 3.8)
project(roskit_bridge_rs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Option to build Rust from source or use pre-built binary
option(BUILD_RUST_FROM_SOURCE "Build Rust bridge from source" ON)

if(BUILD_RUST_FROM_SOURCE)
  # Find Cargo
  find_program(CARGO_EXECUTABLE cargo HINTS $ENV{HOME}/.cargo/bin)

  if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR "Cargo not found. Please install Rust: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
  endif()

  # Build Rust bridge
  set(RUST_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../roskit-bridge-rs")
  set(RUST_TARGET_DIR "${RUST_SOURCE_DIR}/target/release")
  set(RUST_BINARY "${RUST_TARGET_DIR}/roskit-bridge")

  # Custom command to build Rust project
  add_custom_command(
    OUTPUT ${RUST_BINARY}
    COMMAND ${CARGO_EXECUTABLE} build --release
    WORKING_DIRECTORY ${RUST_SOURCE_DIR}
    COMMENT "Building roskit-bridge Rust binary..."
    VERBATIM
  )

  add_custom_target(roskit_bridge_rust ALL DEPENDS ${RUST_BINARY})

  # Install the Rust binary
  install(
    PROGRAMS ${RUST_BINARY}
    DESTINATION lib/${PROJECT_NAME}
    RENAME roskit_bridge
  )
else()
  # When BUILD_RUST_FROM_SOURCE=OFF, the install script handles binary installation
  # Just create an empty marker so colcon succeeds
  message(STATUS "BUILD_RUST_FROM_SOURCE=OFF: Binary will be installed by install.sh script")
endif()

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install resource marker
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
