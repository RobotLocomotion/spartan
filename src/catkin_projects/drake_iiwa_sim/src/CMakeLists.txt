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

add_library(iiwa_lcm
        ${PROJECT_INCLUDE_DIR}/drake_iiwa_sim/iiwa_lcm.h
        iiwa_lcm.cc)
target_link_libraries(iiwa_lcm
        drake::drake)

add_library(iiwa_qp_inverse_dynamics_controller
        ${PROJECT_INCLUDE_DIR}/drake_iiwa_sim/iiwa_qp_inverse_dynamics_controller.h
        iiwa_qp_inverse_dynamics_controller.cc)
target_link_libraries(iiwa_qp_inverse_dynamics_controller
        drake::drake)

add_library(plan_types
        ${PROJECT_INCLUDE_DIR}/robot_plan_runner/plan_base.h
        ${PROJECT_INCLUDE_DIR}/robot_plan_runner/trajectory_plan_base.h
        ${PROJECT_INCLUDE_DIR}/robot_plan_runner/joint_space_trajectory_plan.h
        ${PROJECT_INCLUDE_DIR}/robot_plan_runner/task_space_trajectory_plan.h
        joint_space_trajectory_plan.cc
        task_space_trajectory_plan.cc)
target_link_libraries(plan_types
        drake::drake)

add_library(plan_runner
        ${PROJECT_INCLUDE_DIR}/robot_plan_runner/plan_runner.h
        plan_runner.cc)
target_link_libraries(plan_runner
        plan_types
        drake::drake
        ${CMAKE_THREAD_LIBS_INIT})

add_executable(iiwa_sim
        kuka_simulation.cc)
target_link_libraries(iiwa_sim
        iiwa_lcm
        iiwa_qp_inverse_dynamics_controller
        drake::drake
        gflags_shared)

add_executable(plan_types_test
        plan_types_test.cc)
target_link_libraries(plan_types_test
        plan_types
        drake::drake)

add_executable(run_plan_runner
        run_plan_runner.cc)
target_link_libraries(run_plan_runner
        plan_runner
        drake::drake)

# install library
install(TARGETS iiwa_lcm
 iiwa_qp_inverse_dynamics_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install executable
install(TARGETS iiwa_sim
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

