cmake_minimum_required(VERSION 3.5)
project(stereo_image_pipeline_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

#find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_vitis REQUIRED)
find_package(OpenCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport)
#find_package(ament_cuda)  # enable CUDA capabilities, if available
find_package(stereo_image_proc REQUIRED)

include_directories(include)

#if (ROS_VITIS)
#  ament_auto_add_library(${PROJECT_NAME} SHARED
#    src/disparity_fpga_node.cpp)

#endif()

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

if(DEFINED ROS_VITIS)

  set(lib_depends
  "rclcpp"
  "class_loader"
  "image_transport"
  "OpenCV"
  "stereo_image_proc"
  "vitis_common"
  "ament_vitis"
  )

  find_package(vitis_common REQUIRED)
  add_executable(disparity_fpga_node src/disparity_fpga_node.cpp)
  ament_target_dependencies(disparity_fpga_node ${lib_depends})
  target_include_directories(disparity_fpga_node PUBLIC
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include)

  target_link_libraries(disparity_fpga_node ${OpenCL_LIBRARY})
endif()

# targets
#set (install_targets
#  disparity_fpga_node
#)

if (DEFINED ROS_VITIS)
  set (install_targets ${install_targets}
    disparity_fpga_node
  )

endif()  # ROS_VITIS

install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
