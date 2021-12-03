
#ifndef EDGE_GEAR_CHANGE_H_
#define EDGE_GEAR_CHANGE_H_

#include <float.h>

#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/misc.h"

#include <Eigen/Core>

namespace teb_local_planner {

/**
 * @class EdgeGearChange
 * @brief Edge defining the cost function for switching reverse and forward moves
 *
 * @see TebOptimalPlanner::AddEdgesGearChange
 */
class EdgeGearChange : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
public:
  /**
   * @brief Construct edge.
   */
  EdgeGearChange() { this->setMeasurement(0.); }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeShortestPath()");

    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);

    // ORIENTATION
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    _error[0] = diff1.dot(diff2) < 0.0 ? cfg_->robot.gear_change_time : 0.0;

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeShortestPath::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#endif /* EDGE_GEAR_CHANGE_H_ */
