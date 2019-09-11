cmake_minimum_required(VERSION 2.8.3)
project(gb_datahub)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  bica_graph
)

add_message_files(
  FILES
  Menu.msg
  Product.msg
  Shop.msg
)

add_service_files(
   FILES
   GetMenu.srv
   GetShopsList.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CPR_INCLUDE_DIRS}
)


SET(USE_SYSTEM_GTEST ON)
SET(BUILD_CPR_TESTS OFF)
SET(USE_SYSTEM_CURL ON)

add_subdirectory(cpr)
add_subdirectory(json)


set(GB_DATAHUB_SRCS
  src/${PROJECT_NAME}/gb_datahub.cpp
)

set(GB_DATAHUB_HDRS
  include/${PROJECT_NAME}/gb_datahub.h
)

add_library(${PROJECT_NAME} ${GB_DATAHUB_SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CPR_LIBRARIES} nlohmann_json::nlohmann_json)

#install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


add_executable(coffee_shop_delivery src/coffee_shop_delivery.cpp)
add_dependencies(coffee_shop_delivery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coffee_shop_delivery ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(take_the_elevator src/take_the_elevator.cpp)
add_dependencies(take_the_elevator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(take_the_elevator ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_executable(datahub_tests src/datahub_tests.cpp)
add_dependencies(datahub_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(datahub_tests ${catkin_LIBRARIES} ${PROJECT_NAME} )
