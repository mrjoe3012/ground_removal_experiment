cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 20)

project(ground_removal_experiment)

find_package(PCL 1.2 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS program_options)

set(COMMON_INCLUDE_DIR src/common_src)
set(POINT_CATEGORIZATION_INCLUDE_DIR src/point_categorization_src)
set(GROUND_REMOVAL_INCLUDE_DIR src/ground_removal)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${POINT_CATEGORIZATION_INCLUDE_DIR} ${COMMON_INCLUDE_DIR} ${GROUND_REMOVAL_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND COMMON_SOURCE src/common_src/common.cpp src/common_src/point_categories.cpp src/common_src/debug_out.cpp)

list(APPEND POINT_CATEGORIZATION_SOURCE src/point_categorization_src/point_categorization_main.cpp src/point_categorization_src/point_categorization.cpp src/point_categorization_src/edit_record.cpp src/point_categorization_src/edit_stack.cpp ${COMMON_SOURCE})

list(APPEND ALGORITHM_SIMULATION_SOURCE src/algorithm_simulation_src/algorithm_simulation_main.cpp src/algorithm_simulation_src/parameter_set.cpp src/algorithm_simulation_src/algorithm_parameter.cpp src/algorithm_simulation_src/simulation_data.cpp ${COMMON_SOURCE} src/ground_removal/ground_removal.h)

add_executable(point_categorization ${POINT_CATEGORIZATION_SOURCE})
add_executable(algorithm_simulation ${ALGORITHM_SIMULATION_SOURCE})

target_link_libraries(point_categorization ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(algorithm_simulation ${PCL_LIBRARIES} ${Boost_LIBRARIES})

set_property(TARGET point_categorization PROPERTY CXX_STANDARD 20)
set_property(TARGET algorithm_simulation PROPERTY CXX_STANDARD 20)
