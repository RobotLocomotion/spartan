#pragma once
#include <robot_plan_runner/plan_base.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

namespace drake {
namespace robot_plan_runner {

typedef trajectories::PiecewisePolynomial<double> PPType;

// Base class for all plans that track some sort of trajectory, e.g. joint
// space/task space trajectories.
class TrajectoryPlanBase : public PlanBase {
public:
  TrajectoryPlanBase(std::shared_ptr<const RigidBodyTreed> tree,
                     const PPType &q_traj)
      : PlanBase(std::move(tree)), traj_(q_traj) {
    DRAKE_ASSERT(q_traj.cols() == 1);
    traj_d_ = traj_.derivative(1);
  }

  double duration() {
    if (traj_.get_number_of_segments() > 0) {
      return traj_.end_time() - traj_.start_time();
    } else {
      return 0.;
    }
  }

protected:
  PPType traj_;
  PPType traj_d_; // 1st order derivative of traj_
};

} // namespace robot_plan_runner
} // namespace drake