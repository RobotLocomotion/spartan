/*
 * ur_driver.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/ur_driver.h"
#include "ur_modern_driver/do_output.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>
#include <functional>
#include <boost/bind.hpp>

#include <lcm/lcm-cpp.hpp>
#include "urdriver/ur10_command_t.hpp"
#include "bot_core/robot_state_t.hpp"


class RosWrapper {
    protected:
        UrDriver robot_;
        bot_core::robot_state_t last_state;
        std::mutex last_state_mutex;
        urdriver::ur10_command_t last_command;
        bool alive=false;
        std::condition_variable rt_msg_cond_;
        std::condition_variable msg_cond_;
        double max_Velocity_;
	double max_velocity_;
	std::vector<double> joint_offsets_;
        std::string base_frame_;
        std::string tool_frame_;
        lcm::LCM lcm_;

    public:
        ~RosWrapper() {}
        RosWrapper(std::string host, int reverse_port) : robot_(
                    rt_msg_cond_, msg_cond_, host, reverse_port, 0.03, 300){
            std::vector<std::string> joint_names;
            std::string joint_prefix = "";
            joint_names.push_back(joint_prefix + "shoulder_pan_joint");
            joint_names.push_back(joint_prefix + "shoulder_lift_joint");
            joint_names.push_back(joint_prefix + "elbow_joint");
            joint_names.push_back(joint_prefix + "wrist_1_joint");
            joint_names.push_back(joint_prefix + "wrist_2_joint");
            joint_names.push_back(joint_prefix + "wrist_3_joint");
            robot_.setJointNames(joint_names);
            //Using a very high value in order to not limit execution of trajectories being sent from MoveIt!
            max_velocity_ = 10.;
            double servoj_time = 0.008;
            robot_.setServojTime(servoj_time);

            double servoj_lookahead_time = 0.03;
            robot_.setServojLookahead(servoj_lookahead_time);

            double servoj_gain = 300.;
            robot_.setServojGain(servoj_gain);

            //Base and tool frames
            base_frame_ = joint_prefix + "base_link";
            tool_frame_ =  joint_prefix + "tool0_controller";

            if (robot_.start()) {
                //start actionserver

                //subscribe to the data topic of interest
                new std::thread(
                        boost::bind(&RosWrapper::publishRTMsg, this));
            }
        }

        std::vector<double> convertFV(std::vector<float> fv){
            std::vector<double> dv(fv.begin(), fv.end());
            return dv;
        }

        std::vector<float> convertDV(std::vector<double> dv){
            std::vector<float> fv(dv.begin(), dv.end());
            return fv;
        }

        void publishRTMsg() {
            while (true) {
                bot_core::robot_state_t joint_msg;
                joint_msg.num_joints = 6;
                joint_msg.joint_name = robot_.getJointNames();
		std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
		std::unique_lock<std::mutex> locker(msg_lock);
                while (!robot_.rt_interface_->robot_state_->getDataPublished()) {
                    rt_msg_cond_.wait(locker);
                }
                joint_msg.utime = time(0)*1e6;
                joint_msg.joint_position = convertDV(
                    robot_.rt_interface_->robot_state_->getQActual());
                joint_msg.joint_velocity = convertDV(
                    robot_.rt_interface_->robot_state_->getQdActual());
                joint_msg.joint_effort = convertDV(robot_.rt_interface_->robot_state_->getIActual());
                last_state_mutex.lock();
                last_state = joint_msg;
                last_state_mutex.unlock();
                lcm_.publish("EST_ROBOT_STATE", &joint_msg);

                robot_.rt_interface_->robot_state_->setDataPublished();
            }
        }

        bool validateCommand(std::vector<double> new_pos, std::vector<double> old_pos, std::vector<double> old_vel, double max_vel, double max_acc, double dt){
            std::vector<double> new_vel(old_pos.size());
            std::vector<double> new_acc(old_pos.size());
            std::transform(new_pos.begin(), new_pos.end(), old_pos.begin(), new_vel.begin(), std::minus<double>());
            std::transform(new_vel.begin(), new_vel.end(), new_vel.begin(), bind2nd(std::divides<double>(), dt));
            std::transform(new_vel.begin(), new_vel.end(), old_vel.begin(), new_acc.begin(), std::minus<double>());
            std::transform(new_acc.begin(), new_acc.end(), new_acc.begin(), bind2nd(std::divides<double>(), dt));
            for(int i =0; i < new_vel.size(); i ++){
                if(new_vel[i] > max_vel || new_acc[i] > max_acc)
                    return false;
            }
            return true;
        }

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const urdriver::ur10_command_t* msg){
            std::vector<double> positions = {};
            if(alive)
                positions = msg->p;
            std::vector<double> last_commanded_positions = last_command.p;
            last_state_mutex.lock();
            std::vector<double> last_actual_positions = convertFV(last_state.joint_position);
            std::vector<double> last_actual_velocity = convertFV(last_state.joint_velocity);
            last_state_mutex.unlock();

            if((!alive || validateCommand(positions, last_commanded_positions, last_actual_velocity, 10.0, 4.0, 0.008))
                    && validateCommand(positions, last_actual_positions, last_actual_velocity, 10.0, 4.0, 0.008))
                robot_.servoj(positions, msg->keepalive);
            else
                print_fatal("Commanded message contained a point too far from current motion");
        }

        };
class Handler{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const urdriver::ur10_command_t* msg)
        {}
};

        int main(int argc, char **argv) {
            bool use_sim_time = false;
            if(argc == 1){
                print_fatal("Please provide robot ip.");
                exit(1);
            }
            std::string host=argv[1];
            int reverse_port = 50001;

            lcm::LCM lcm;

            RosWrapper interface(host, reverse_port);
            lcm.subscribe("ROBOT_COMMAND", &RosWrapper::handleMessage, &interface);
            while(0 == lcm.handle());

            exit(0);
        }
