cmake_minimum_required(VERSION 3.5)
project(moveit_task_constructor_ur5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	rclcpp
	tf2_eigen
)

add_library(${PROJECT_NAME}_pick_place_task SHARED src/pick_place_task.cpp)
ament_target_dependencies(${PROJECT_NAME}_pick_place_task ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_library(${PROJECT_NAME}_pick_place_task_solo SHARED src/pick_place_task_solo.cpp)
ament_target_dependencies(${PROJECT_NAME}_pick_place_task_solo ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS ${PROJECT_NAME}_pick_place_task
				${PROJECT_NAME}_pick_place_task_solo
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME})

include_directories(include)

# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
	set(parameter_filename ${name}_parameters)
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${parameter_filename}.yaml)
		generate_parameter_library(${parameter_filename} src/${parameter_filename}.yaml)
		target_link_libraries(${PROJECT_NAME}_${name} ${parameter_filename})
	endif()
	ament_target_dependencies(${PROJECT_NAME}_${name} ${THIS_PACKAGE_INCLUDE_DEPENDS})
	set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
	install(TARGETS ${PROJECT_NAME}_${name}
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION lib/${PROJECT_NAME}
	)
endfunction()

# # demo(cartesian)
# # demo(modular)
# # demo(alternative_path_costs)
# # demo(ik_clearance_cost)
# # demo(fallbacks_move_to)

demo(pick_place_ur5)
target_link_libraries(${PROJECT_NAME}_pick_place_task pick_place_ur5_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_ur5 ${PROJECT_NAME}_pick_place_task)
demo(pick_place_ur5_solo)
target_link_libraries(${PROJECT_NAME}_pick_place_task_solo pick_place_ur5_solo_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_ur5_solo ${PROJECT_NAME}_pick_place_task_solo)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# TODO: Port tests
# add_subdirectory(test)

ament_package()
