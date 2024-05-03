cmake_minimum_required(VERSION 2.8.3)
project(ltl_project)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS xacro
        moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package()

add_executable(pick_and_place src/pick_and_place.cpp)
add_executable(pick src/pick.cpp)
add_executable(place src/place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES})
target_link_libraries(pick ${catkin_LIBRARIES})
target_link_libraries(place ${catkin_LIBRARIES})
add_dependencies(pick_and_place ${catkin_EXPORTED_TARGETS})
add_dependencies(pick ${catkin_EXPORTED_TARGETS})
add_dependencies(place ${catkin_EXPORTED_TARGETS})

install(TARGETS pick_and_place
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS pick
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS place
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

file(GLOB xacro_files model/robots/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
