#include <drake_robot_control/plan_base.h>
#include <chrono>


namespace drake {
namespace robot_plan_runner {

PlanStatus PlanBase::WaitForPlanToFinish(){

    std::unique_lock<std::mutex> lock(mutex_); // this acquires the lock
    if(is_finished_ == false){
        // loop until the plan is finished, wake up every 500ms to check if plan is finished
        while (true){
            if (cv_.wait_for(lock, std::chrono::milliseconds(500), [&](){return is_finished_;})){
                break;
            }
        }    
    }
    return this->get_plan_status();
}

void PlanBase::SetPlanFinished(){
    std::unique_lock<std::mutex> lock(mutex_);
    is_finished_ = true;
    lock.unlock();
    cv_.notify_all();
}

void PlanBase::GetPlanStatusMsg(robot_msgs::PlanStatus& plan_status_msg){
    PlanStatus plan_status = this->get_plan_status();

    switch(plan_status) {
        case PlanStatus::FINISHED_NORMALLY:{
          plan_status_msg.status = plan_status_msg.FINISHED_NORMALLY;
          break;
        } 
        case PlanStatus::STOPPED_BY_EXTERNAL_TRIGGER: {
          plan_status_msg.status = plan_status_msg.STOPPED_EXTERNALLY;
          break;
        }
        default: {
          plan_status_msg.status = plan_status_msg.ERROR;
          break;
        }
    }
}

} // namespace robot_plan_runner
} // namespace drake
