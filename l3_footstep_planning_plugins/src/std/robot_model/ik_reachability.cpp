#include <l3_footstep_planning_plugins/std/robot_model/ik_reachability.h>

#include <l3_plugins/robot_model.h>

namespace l3_footstep_planning
{
IKReachability::IKReachability()
  : ReachabilityPlugin("ik_reachability")
{}

bool IKReachability::loadParams(const ParameterSet& params)
{
  if (!ReachabilityPlugin::loadParams(params))
    return false;

  // read resolution
  if (hasParam("resolution"))
    res_ = DiscreteResolution(getSubset("resolution"));
  // use planner resolution
  else
    res_ = DiscreteResolution(params.getSubset("resolution"));

  foot_idx_ = param("foot_idx", 0u);

  FootInfo foot_info;
  if (!RobotModel::description()->getFootInfo(foot_idx_, foot_info))
  {
    ROS_ERROR("[%s] Foot idx '%u' is unknown!", getName().c_str(), foot_idx_);
    return false;
  }
  neutral_stance_ = foot_info.neutral_stance;

  // get joint limits
  joint_limits_.clear();

  XmlRpc::XmlRpcValue p;
  if (getParam("limits", p, XmlRpc::XmlRpcValue(), true))
  {
    LegInfo leg_info;

    if (!RobotModel::description()->getLegInfo(foot_idx_, leg_info))
    {
      ROS_ERROR("[%s] Leg idx '%u' is unknown!", getName().c_str(), foot_idx_);
      return false;
    }

    joint_limits_.resize(leg_info.joints.size());

    for (size_t i = 0; i < leg_info.joints.size(); i++)
    {
      const std::string& name = leg_info.joints[i];

      if (p.hasMember(name))
      {
        XmlRpc::XmlRpcValue limits = p[name];
        if (limits.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          joint_limits_[i].first = static_cast<double>(limits[0]);
          joint_limits_[i].second = static_cast<double>(limits[1]);
        }
        else
          ROS_ERROR("[%s] Joint limit for '%s' is not given as array.", getName().c_str(), name.c_str());
      }
      else
      {
        joint_limits_[i].first = -std::numeric_limits<double>::max();
        joint_limits_[i].second = std::numeric_limits<double>::max();
      }
    }
  }

  // get search limits
  std::vector<double> limits;
  if (!getParam("x", limits, std::vector<double>()) || limits.size() != 2)
    return false;
  else
  {
    min_x_ = res_.toDiscX(neutral_stance_.x() + limits[0]);
    max_x_ = res_.toDiscX(neutral_stance_.x() + limits[1]);
  }

  if (!getParam("y", limits, std::vector<double>()) || limits.size() != 2)
    return false;
  else
  {
    min_y_ = res_.toDiscY(neutral_stance_.y() + limits[0]);
    max_y_ = res_.toDiscY(neutral_stance_.y() + limits[1]);
  }

  if (!getParam("z", limits, std::vector<double>()) || limits.size() != 2)
    return false;
  else
  {
    min_z_ = res_.toDiscZ(neutral_stance_.z() + limits[0]);
    max_z_ = res_.toDiscZ(neutral_stance_.z() + limits[1]);
  }

  if (!getParam("yaw", limits, std::vector<double>(), true))
  {
    min_yaw_ = 0;
    max_yaw_ = 0;
  }
  else if (limits.size() == 2)
  {
    min_yaw_ = res_.toDiscAngle(neutral_stance_.yaw() + limits[0]);
    max_yaw_ = res_.toDiscAngle(neutral_stance_.yaw() + limits[1]);
  }
  else
  {
    ROS_ERROR("[%s] Yaw limits must be given as array with two entries!", getName().c_str());
    return false;
  }

  use_sway_ = param("use_sway", false, true);

  return true;
}

bool IKReachability::initialize(const ParameterSet& params)
{
  if (!ReachabilityPlugin::initialize(params))
    return false;

  if (!RobotModel::kinematics())
  {
    ROS_ERROR("[%s] KinematicsPlugin required!", getName().c_str());
    return false;
  }

  FootholdArray footholds;
  for (Foothold::ConstPtr f : RobotModel::description()->getNeutralStance())
  {
    if (f->idx != foot_idx_)
      footholds.push_back(*f);
  }

  Transform center_to_base = RobotModel::kinematics()->calcStaticFeetCenterToBase(*RobotModel::description());

  bool print_rhm = param("print_rhm", false, true);
  std::string csv_file = param("csv_file", std::string(), true);

  // generate reachability height maps
  size_t width = max_x_ - min_x_ + 1;
  size_t height = max_y_ - min_y_ + 1;

  std::vector<double> q_init;
  RobotModel::kinematics()->calcLegIK(Foothold(foot_idx_, neutral_stance_), *RobotModel::description(), q_init);

  rhm_positive_.init(width, height);
  rhm_negative_.init(width, height);

  for (int y = 0; y < height; y++)
  {
    //ROS_INFO("[%s] %i / %lu", getName().c_str(), y, height);
    for (int x = 0; x < width; x++)
    {
      rhm_positive_.at(x, y) = -std::numeric_limits<float>::max();
      rhm_negative_.at(x, y) = std::numeric_limits<float>::max();
      bool found_min = false;

      for (int z = min_z_; z <= max_z_; z++)
      {
        double cont_z = res_.toContZ(z);
        // sample area
        // for (int yaw = min_yaw_; yaw <= max_yaw_; yaw++)
        {
          std::vector<double> q;

          Foothold foothold(foot_idx_, res_.toContX(min_x_ + x), res_.toContY(min_y_ + y), cont_z /*, res_.toContYaw(min_yaw_ + yaw)*/);

          if (use_sway_)
          {
            FootholdArray feet = footholds;
            feet.push_back(foothold);

            Pose base_pose = RobotModel::kinematics()->calcFeetCenter(feet) * center_to_base;

            if (!RobotModel::kinematics()->calcLegIK(base_pose, foothold, *RobotModel::description(), q_init, q))
            {
              // if (found_min)
              //  z = max_z_;  // terminate current loop
              continue;
            }
          }
          else
          {
            if (!RobotModel::kinematics()->calcLegIK(foothold, *RobotModel::description(), q_init, q))
            {
              // if (found_min)
              //  z = max_z_;  // terminate current loop
              continue;
            }
          }

          // check joint limits
          ROS_ASSERT(joint_limits_.empty() || joint_limits_.size() == q.size());
          bool in_limit = true;
          for (size_t i = 0; i < joint_limits_.size(); i++)
          {
            if (q[i] < joint_limits_[i].first || q[i] > joint_limits_[i].second)
            {
              // ROS_WARN("[%lu] %.2f - %.2f - %.2f", i, joint_limits_[i].first, q[i], joint_limits_[i].second);
              in_limit = false;
              break;
            }
          }
          if (!in_limit)
            continue;

          /// @todo Check Jacobian and define limits next to singularities

          if (!found_min)
          {
            rhm_negative_.at(x, y) = static_cast<float>(cont_z);
            found_min = true;
          }

          rhm_positive_.at(x, y) = static_cast<float>(cont_z);
        }
      }
    }
  }

  ROS_INFO_STREAM_COND(print_rhm, "RHMs [" << foot_idx_ << "]:\n" << toString(rhm_positive_, "RHM+") << "\n" << toString(rhm_negative_, "RHM-"));

  if (!csv_file.empty())
  {
    /// @todo handle suffix better
    toCSV(csv_file + "_" + std::to_string(foot_idx_) + "_pos.csv", rhm_positive_);
    toCSV(csv_file + "_" + std::to_string(foot_idx_) + "_neg.csv", rhm_negative_);
  }

  return true;
}  // namespace l3_footstep_planning

bool IKReachability::isReachable(const PlanningState& state) const
{
  FootStepData::ConstPtr foot_step = state.getStep()->footStep().getMovingLink(foot_idx_);
  if (foot_step)
    return isReachable(state.getState()->getFeetCenter(), *foot_step->target);

  return true;
}

bool IKReachability::isReachable(const State& state) const
{
  Foothold::ConstPtr foothold = state.getFoothold(foot_idx_);
  if (foothold)
    return isReachable(state.getFeetCenter(), *foothold);

  return true;
}

bool IKReachability::isReachable(const Pose& feet_center, const Foothold& foothold) const
{
  ROS_ASSERT(foothold.idx == foot_idx_);

  // get delta
  Transform delta = Transform::getTransform(foothold.pose(), feet_center);

  int disc_dx = res_.toDiscX(delta.x()) - min_x_;
  if (disc_dx >= rhm_positive_.width())
    return false;

  int disc_dy = res_.toDiscY(delta.y()) - min_y_;
  if (disc_dy >= rhm_positive_.height())
    return false;

  // check if in range
  return delta.z() >= rhm_negative_.at(disc_dx, disc_dy) && delta.z() <= rhm_positive_.at(disc_dx, disc_dy);
}

std::string IKReachability::toString(const Array2D<float>& array, const std::string& name) const
{
  std::string msg;

  msg += "--- " + name + " ---\n";

  for (int y = array.height() - 1; y >= 0; y--)
  {
    if (y >= 0)
      msg += " ";
    if (std::abs(y) <= 9)
      msg += " ";
    msg += std::to_string(y) + ": ";

    for (int x = 0; x < array.width(); x++)
    {
      if (std::abs(array.at(x, y)) == std::numeric_limits<float>::max())
        msg += "    x    ";
      else
        msg += std::to_string(array.at(x, y)) + " ";
    }

    msg += "\n";
  }

  return msg;
}

bool IKReachability::toCSV(const std::string file, const Array2D<float>& array) const
{
  std::ofstream outfile;
  outfile.open(file);

  if (!outfile.is_open())
  {
    ROS_ERROR("[%s] Could not write to '%s'!", getName().c_str(), file.c_str());
    return false;
  }

  for (int y = 0; y < array.height(); y++)
  {
    for (int x = 0; x < array.width(); x++)
    {
      if (std::abs(array.at(x, y)) == std::numeric_limits<float>::max())
        outfile << "NaN";
      else
        outfile << std::to_string(array.at(x, y));

      if (x < (array.width() - 1))
        outfile << ",";
    }

    outfile << "\n";
  }

  outfile.close();

  return true;
}
}  // namespace l3_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_footstep_planning::IKReachability, l3_footstep_planning::ReachabilityPlugin)
