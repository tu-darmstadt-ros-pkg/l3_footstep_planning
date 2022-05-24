
#include <l3_footstep_planning_plugins/aggregator/hfs_heuristic.h>

namespace l3_footstep_planning
{
using namespace l3;

HFSHeuristic::HFSHeuristic()
  : ExtendedPluginAggregator<HFSHeuristic, HFSHeuristicPlugin>("HFSHeuristic")
{}

bool HFSHeuristic::getHardest(l3::ExpandStatesIdx& next, const l3::ExpandStatesIdx& idxs, Step::ConstPtr step) const
{
  std::vector<double> feetCosts(idxs.foot_idx.size(), 0);
  std::vector<double> feetRisk(idxs.foot_idx.size(), 0);
  std::vector<double> baseCosts(idxs.floating_base_idx.size(), 0);
  std::vector<double> baseRisk(idxs.floating_base_idx.size(), 0);

  int num_plugins = 0;
  for (HFSHeuristicPlugin::ConstPtr plugin : getPlugins())  // iterate through all plugins
  {
    num_plugins++;
    for (auto i = 0; i < (int)idxs.foot_idx.size(); i++)
    {
      double cost;
      double risk;
      if (!plugin->getFeetHeuristic(idxs.foot_idx[i], step, cost, risk))
      {
        ROS_WARN_ONCE("No Heuristic given for foot index: %d", idxs.foot_idx[i]);
        next.foot_idx = FootIndexArray{ idxs.foot_idx[i] };
        return false;
      }
      feetCosts[i] += cost;
      feetRisk[i] += risk;
    }

    for (auto i = 0; i < (int)idxs.floating_base_idx.size(); i++)
    {
      double cost;
      double risk;
      if (!plugin->getBaseHeuristic(idxs.floating_base_idx[i], step, cost, risk))
      {
        ROS_WARN_ONCE("No Heuristic given for foot index: %d", idxs.foot_idx[i]);
      }
      else
      {
        baseCosts[i] += cost;
        baseRisk[i] += risk;
      }
    }
  }

  // no plugins or no costs given, then return all the idcs
  if (num_plugins == 0 or (feetCosts.empty() and baseCosts.empty()))
  {
    next = idxs;
    return false;
  }

  // viable costs are per definition between 0 (easiest) and 1 (hardest)
  double max_feet_cost = -1;
  double max_base_cost = -1;
  if (!feetCosts.empty())
  {
    max_feet_cost = *std::max_element(feetCosts.begin(), feetCosts.end());
  }
  if (!baseCosts.empty())
  {
    max_base_cost = *std::max_element(baseCosts.begin(), baseCosts.end());
  }
  if (max_feet_cost < max_base_cost)
  {
    next.floating_base_idx =
        BaseIndexArray{ idxs.floating_base_idx[std::max_element(baseCosts.begin(), baseCosts.end()) - baseCosts.begin()] };  // get base index with highest cost
  }
  else
  {
    next.foot_idx = FootIndexArray{ idxs.foot_idx[std::max_element(feetCosts.begin(), feetCosts.end()) - feetCosts.begin()] };  // get foot index with highest cost
  }
  return true;
}
}  // namespace l3_footstep_planning
