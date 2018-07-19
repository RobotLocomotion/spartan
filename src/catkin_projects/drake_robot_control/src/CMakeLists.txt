# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2017, Massachusetts Institute of Technology.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

add_library(plan_types
        ${PROJECT_INCLUDE_DIR}/drake_robot_control/plan_base.h
        ${PROJECT_INCLUDE_DIR}/drake_robot_control/trajectory_plan_base.h
        ${PROJECT_INCLUDE_DIR}/drake_robot_control/joint_space_trajectory_plan.h
        ${PROJECT_INCLUDE_DIR}/drake_robot_control/task_space_trajectory_plan.h
        joint_space_trajectory_plan.cc
        task_space_trajectory_plan.cc
        plan_base.cc)

# following http://docs.ros.org/jade/api/catkin/html/howto/format2/cpp_msg_dependencies.html
add_dependencies(plan_types ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_types
        drake::drake
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})

add_library(plan_runner
        ${PROJECT_INCLUDE_DIR}/drake_robot_control/plan_runner.h
        plan_runner.cc)
add_dependencies(plan_runner ${catkin_EXPORTED_TARGETS})

target_link_libraries(plan_runner
        plan_types
        drake::drake
        ${CMAKE_THREAD_LIBS_INIT}
        ${catkin_LIBRARIES})

#add_executable(test_plan_types
#        test_plan_types.cc)
#target_link_libraries(test_plan_types
#        plan_types
#        drake::drake)

add_executable(test_plan_runner
        test_plan_runner.cc)
target_link_libraries(test_plan_runner
        plan_runner
        drake::drake)

add_executable(plan_runner_node
        plan_runner_node.cc)
add_dependencies(plan_runner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_runner_node
        gflags_shared
        plan_runner
        ${catkin_LIBRARIES})

# install library
install(TARGETS plan_types plan_runner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install executable
install(TARGETS plan_runner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

