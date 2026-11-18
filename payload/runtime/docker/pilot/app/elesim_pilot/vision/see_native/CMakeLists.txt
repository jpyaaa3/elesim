cmake_minimum_required(VERSION 3.19)
project(elesim_see LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PCL 1.12 REQUIRED COMPONENTS common octree)
find_package(OpenMP REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)
find_library(LEMON_LIBRARY lemon REQUIRED)
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/prepare.py
  ${CMAKE_CURRENT_BINARY_DIR}/adapted COMMAND_ERROR_IS_FATAL ANY)
add_library(elesim_see SHARED bridge.cpp)
target_include_directories(elesim_see PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/adapted
  ${CMAKE_CURRENT_BINARY_DIR}/adapted/see_core/include
  ${CMAKE_CURRENT_BINARY_DIR}/adapted/see_common/include ${PCL_INCLUDE_DIRS})
target_link_libraries(elesim_see PRIVATE ${PCL_LIBRARIES} ${NLOPT_LIBRARY}
  ${LEMON_LIBRARY} OpenMP::OpenMP_CXX)
install(TARGETS elesim_see LIBRARY DESTINATION lib)
