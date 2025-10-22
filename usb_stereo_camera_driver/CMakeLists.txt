cmake_minimum_required(VERSION 3.8)
project(usb_stereo_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  include
)

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(driver src/main.cpp src/driver.cpp)
ament_target_dependencies(driver rclcpp sensor_msgs cv_bridge)
target_link_libraries(driver ${OpenCV_LIBRARIES})

install(
  TARGETS driver
  DESTINATION lib/${PROJECT_NAME}  
)

add_library(driver_component SHARED src/composable_main.cpp src/driver.cpp)
ament_target_dependencies(driver_component rclcpp rclcpp_components sensor_msgs cv_bridge)
target_link_libraries(driver_component ${OpenCV_LIBRARIES})

rclcpp_components_register_node(
  driver_component
  PLUGIN "stereoCamera::UsbStereoCameraDriver"
  EXECUTABLE composable_driver
)

ament_export_targets(export_driver_component)
install(TARGETS driver_component
  EXPORT export_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
