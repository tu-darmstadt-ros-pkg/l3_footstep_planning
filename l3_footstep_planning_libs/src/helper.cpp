#include <l3_footstep_planning_libs/helper.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_libs/modeling/floating_base_id.h>

namespace l3_footstep_planning
{
bool getXYZ(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val)
{
  if (!nh.hasParam(name + "/x") || !nh.hasParam(name + "/y") || !nh.hasParam(name + "/z"))
  {
    ROS_ERROR("Couldn't retrieve parameter '%s' as Vector3", (nh.getNamespace() + "/" + name).c_str());
    return false;
  }

  nh.getParam(name + "/x", val.x);
  nh.getParam(name + "/y", val.y);
  nh.getParam(name + "/z", val.z);
  return true;
}

bool getRPY(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val)
{
  if (!nh.hasParam(name + "/roll") || !nh.hasParam(name + "/pitch") || !nh.hasParam(name + "/yaw"))
  {
    ROS_ERROR("Couldn't retrieve parameter '%s' as RPY orienation", (nh.getNamespace() + "/" + name).c_str());
    return false;
  }

  nh.getParam(name + "/roll", val.x);
  nh.getParam(name + "/pitch", val.y);
  nh.getParam(name + "/yaw", val.z);
  return true;
}

bool getGridMapCoords(const nav_msgs::OccupancyGrid& map, double x, double y, int& map_x, int& map_y)
{
  map_x = round((x - map.info.origin.position.x) / map.info.resolution);
  map_y = round((y - map.info.origin.position.y) / map.info.resolution);

  if (map_x < 0 || (int)map.info.width <= map_x || map_y < 0 || (int)map.info.height <= map_y)
    return false;

  return true;
}

bool getGridMapIndex(const nav_msgs::OccupancyGrid& map, double x, double y, int& idx)
{
  int map_x, map_y;

  if (!getGridMapCoords(map, x, y, map_x, map_y))
    return false;
  else
    idx = map_x + map_y * map.info.width;

  return true;
}

FootIndexArray applyFootIdxWhitelist(const FootIndexArray& foot_idx, const FootIndexSet& foot_idx_whitelist)
{
  if (foot_idx_whitelist.empty())
    return foot_idx;
  else
  {
    FootIndexArray result;
    for (const FootIndex& idx : foot_idx)
    {
      if (foot_idx_whitelist.find(idx) != foot_idx_whitelist.end())
        result.push_back(idx);
    }
    return result;
  }
}

l3_msgs::FootholdArray applyFootIdxWhitelist(const l3_msgs::FootholdArray& footholds, const FootIndexSet& foot_idx_whitelist)
{
  if (foot_idx_whitelist.empty())
    return footholds;
  else
  {
    l3_msgs::FootholdArray result;
    for (const l3_msgs::Foothold& f : footholds)
    {
      if (foot_idx_whitelist.find(f.idx) != foot_idx_whitelist.end())
        result.push_back(f);
    }
    return result;
  }
}

FootholdArray applyFootIdxWhitelist(const FootholdArray& footholds, const FootIndexSet& foot_idx_whitelist)
{
  if (foot_idx_whitelist.empty())
    return footholds;
  else
  {
    FootholdArray result;
    for (const Foothold& f : footholds)
    {
      if (foot_idx_whitelist.find(f.idx) != foot_idx_whitelist.end())
        result.push_back(f);
    }
    return result;
  }
}

FootholdPtrArray applyFootIdxWhitelist(const FootholdPtrArray& footholds, const FootIndexSet& foot_idx_whitelist)
{
  if (foot_idx_whitelist.empty())
    return footholds;
  else
  {
    FootholdPtrArray result;
    for (Foothold::Ptr f : footholds)
    {
      if (foot_idx_whitelist.find(f->idx) != foot_idx_whitelist.end())
        result.push_back(f);
    }
    return result;
  }
}

FootholdConstPtrArray applyFootIdxWhitelist(const FootholdConstPtrArray& footholds, const FootIndexSet& foot_idx_whitelist)
{
  if (foot_idx_whitelist.empty())
    return footholds;
  else
  {
    FootholdConstPtrArray result;
    for (Foothold::ConstPtr f : footholds)
    {
      if (foot_idx_whitelist.find(f->idx) != foot_idx_whitelist.end())
        result.push_back(f);
    }
    return result;
  }
}

FootholdPtrArray getNeutralStance(const FloatingBase& floating_base)
{
  ROS_ASSERT(l3::RobotModel::kinematics());

  l3::Pose feet_center = floating_base.pose() * l3::RobotModel::kinematics()->calcStaticFeetCenterToBase(*l3::RobotModel::description()).inverse();

  return RobotModel::getNeutralStance(feet_center);
}

FootholdPtrArray getNeutralStance(const FloatingBase& floating_base, const DiscreteResolution& resolution)
{
  FloatingBaseID id(floating_base, resolution);
  return getNeutralStance(id.getDiscreteFloatingBase(resolution));
}
}  // namespace l3_footstep_planning
