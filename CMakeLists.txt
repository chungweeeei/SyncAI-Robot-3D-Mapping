cmake_minimum_required(VERSION 3.8)
project(syncai_mapping)

# Unoptimised, iSAM2 + loop GICP + the map merge cannot keep up on the Jetson.
# Default to Release, but let an explicit CMAKE_BUILD_TYPE through.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Carried over from upstream; the map-cloud merge runs on a raw std::thread.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
# pgo_node locates the clean_map executable it spawns through the package index.
find_package(ament_index_cpp REQUIRED)
# StartMapping / SaveMaps / ResetMapping (served) and ResetLIO (client).
find_package(syncai_common REQUIRED)

# GTSAM: source-built into /usr/local by the Dockerfile, no rosdep key (see
# package.xml). Eigen / PCL are found explicitly for ${PCL_LIBRARIES} and the
# include dirs GTSAM's headers need.
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# Loop verification (RegistrationPCL). A plain cmake package from
# src/third-party, ordered first by <depend>small_gicp</depend>.
find_package(small_gicp REQUIRED)
# clean_map: voxel keys, ray traversal and the optional octomap.bt. From apt
# (ros-humble-octomap): a source checkout would also build octovis and
# dynamicEDT3D.
find_package(octomap REQUIRED)
# The map_cloud_file notice and map_clean.recipe.json. Header-only, from apt.
find_package(nlohmann_json 3.2.0 REQUIRED)

set(SYNCAI_MAPPING_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
  message_filters
  syncai_common
)

# ---- The post-save map cleaning: sidecar, cleaner, CLI ------------------------
# Split so pgo_node links neither OctoMap nor the ray-cast: it only writes the
# "converting" sidecar and spawns clean_map.
add_library(dynamic_removal_recipe STATIC src/dynamic_removal/recipe.cpp)
target_include_directories(dynamic_removal_recipe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dynamic_removal_recipe PRIVATE nlohmann_json::nlohmann_json)

add_library(dynamic_removal STATIC src/dynamic_removal/remover.cpp)
target_include_directories(dynamic_removal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(dynamic_removal PUBLIC ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(clean_map src/dynamic_removal/main.cpp)
target_link_libraries(clean_map dynamic_removal dynamic_removal_recipe)

# ---- Ground patches: the console's octomap_edits.json applied to octomap.bt --
# Its own library: OctoMap and JSON, no PCL, so the tool builds standalone.
add_library(dynamic_removal_edits STATIC src/dynamic_removal/edits.cpp)
target_include_directories(dynamic_removal_edits PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(dynamic_removal_edits
  PUBLIC ${OCTOMAP_LIBRARIES}
  PRIVATE nlohmann_json::nlohmann_json)

add_executable(apply_octomap_edits src/dynamic_removal/apply_edits_main.cpp)
target_link_libraries(apply_octomap_edits dynamic_removal_edits)

# ---- pgo_node: the online pose graph (mapping session) ----------------------
# The pose graph is ROS-free, so the unit tests link it too.
add_library(pgos STATIC
  src/pgos/commons.cpp
  src/pgos/pose_graph_optimizer.cpp
  src/pgos/scan_context.cpp
)
target_include_directories(pgos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
# small_gicp::small_gicp carries OpenMP; without it its pragmas are silently
# ignored and it runs single-threaded.
target_link_libraries(pgos PUBLIC
  ${PCL_LIBRARIES}
  gtsam
  small_gicp::small_gicp
)

add_executable(pgo_node
  src/main.cpp
  src/pgo_node.cpp
)
ament_target_dependencies(pgo_node ${SYNCAI_MAPPING_DEPS} ament_index_cpp)
target_link_libraries(pgo_node
  pgos
  dynamic_removal_recipe
  nlohmann_json::nlohmann_json
)

install(TARGETS pgo_node clean_map apply_octomap_edits
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # pgos/ is upstream code without per-file licence headers (LICENSE covers
  # it); copyright and cpplint are off as upstream had them.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # .clang-format (checked in CI) and uncrustify's ROS style contradict each other.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_context test/test_scan_context.cpp)
  target_link_libraries(test_scan_context pgos)
  # Loop closure end to end on a synthetic building. Slow-ish.
  ament_add_gtest(test_loop_closure test/test_loop_closure.cpp TIMEOUT 300)
  target_link_libraries(test_loop_closure pgos)
  ament_add_gtest(test_octomap_edits test/test_octomap_edits.cpp)
  target_link_libraries(test_octomap_edits dynamic_removal_edits)
endif()

ament_package()
