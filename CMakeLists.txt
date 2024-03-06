# Sets the minimum required CMake version for this project to 3.16
cmake_minimum_required(VERSION 3.16)
# Defines the project named vacuum_cleaner and specifies that the C++ (CXX) programming language is used
project(vacuum_cleaner LANGUAGES CXX)

# Adds compiler options for GNU or Clang, particularly warnings (-Wall and -Wextra)
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Finds and uses ament_cmake, the package for CMake modules in ROS 2
find_package(ament_cmake REQUIRED)

# Sets a list of dependencies for this package (project). These dependencies will be found and used later
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  jsoncpp
)

# Iterates over the dependencies defined in THIS_PACKAGE_INCLUDE_DEPENDS and finds them using find_package
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Adds a library named vacuum_cleaner, created from the source file hardware/diffbot_system.cpp
add_library(
  vacuum_cleaner
  SHARED
  hardware/diffbot_system.cpp
  hardware/arduino_com.cpp
)

# Specifies that C++ standard features from C++17 should be used for the vacuum_cleaner library
target_compile_features(vacuum_cleaner PUBLIC cxx_std_17)

# Configures the include directories for the vacuum_cleaner library. 
# The hardware/include directory is used for the build, and include/vacuum_cleaner is used for installation
target_include_directories(vacuum_cleaner PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/vacuum_cleaner>
)
# Specifies the ROS 2 dependencies for the vacuum_cleaner library.
ament_target_dependencies(
  vacuum_cleaner PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Links the vacuum_cleaner library with the serial library.
target_link_libraries(vacuum_cleaner PUBLIC serial)
target_link_libraries(vacuum_cleaner PUBLIC jsoncpp_lib)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "vacuum_cleaner_BUILDING_DLL")

target_compile_options(vacuum_cleaner PUBLIC -ljsoncpp)

# Exports a plugin description file for the hardware_interface.
pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

# Installs necessary fildes to the target directorys
install(
  DIRECTORY config
  DESTINATION share/vacuum_cleaner
)
install(
  DIRECTORY description
  DESTINATION share/vacuum_cleaner
)
install(
  DIRECTORY hardware/include/
  DESTINATION include/vacuum_cleaner
)
install(
  DIRECTORY launch
  DESTINATION share/vacuum_cleaner
)
install(
  DIRECTORY worlds
  DESTINATION share/vacuum_cleaner
)
install(
  DIRECTORY maps
  DESTINATION share/vacuum_cleaner
)
install(TARGETS vacuum_cleaner
  EXPORT export_vacuum_cleaner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# include python scripts
install(PROGRAMS
  scripts/cleaning.py
  DESTINATION lib/${PROJECT_NAME})

# If the BUILD_TESTING option is enabled, finds and uses the GTest package
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# Exports ROS 2 dependencies and configures the ROS 2 package.
ament_export_targets(export_vacuum_cleaner HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
