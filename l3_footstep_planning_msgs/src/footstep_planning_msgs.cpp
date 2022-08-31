#include <l3_footstep_planning_msgs/footstep_planning_msgs.h>

#include <l3_libs/types/types.h>

namespace l3_footstep_planning
{
// Extension to ErrorStatus message
msgs::ErrorStatus operator+(const msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs)
{
  msgs::ErrorStatus result;

  result.error = lhs.error | rhs.error;
  result.error_msg = lhs.error_msg;
  if (result.error_msg.size() && rhs.error_msg.size())
    result.error_msg += "\n";
  result.error_msg += rhs.error_msg;

  result.warning = lhs.warning | rhs.warning;
  result.warning_msg = lhs.warning_msg;
  if (result.warning_msg.size() && rhs.warning_msg.size())
    result.warning_msg += "\n";
  result.warning_msg += rhs.warning_msg;

  return result;
}

msgs::ErrorStatus operator+=(msgs::ErrorStatus& lhs, const msgs::ErrorStatus& rhs)
{
  lhs = operator+(lhs, rhs);
  return lhs;
}

msgs::ErrorStatus isConsistent(const msgs::StepPlan& step_plan)
{
  // an empty plan is always consistent
  if (step_plan.plan.steps.empty())
    return msgs::ErrorStatus();

  msgs::ErrorStatus status;

  // check if steps are ordered and closed
  l3::StepIndex step_idx = -1;

  for (const msgs::Step& step : step_plan.plan.steps)
  {
    if (step.foot_steps.empty() && step.support_feet.empty())
    {
      status += ErrorStatusWarning(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, "isConsistent", "Step plan contains empty steps!");
      continue;
    }

    if (step_idx == -1)
      step_idx = step.idx;

    for (const msgs::FootStepData& foot_step : step.foot_steps)
    {
      if (foot_step.origin.header.frame_id != step_plan.header.frame_id)
        status += ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, "isConsistent",
                                   "Frame id mismatch of step " + boost::lexical_cast<std::string>(step.idx) + " Plan: '" + step_plan.header.frame_id + "' vs. step origin: '" +
                                       foot_step.origin.header.frame_id + "'");

      if (foot_step.target.header.frame_id != step_plan.header.frame_id)
        status += ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, "isConsistent",
                                   "Frame id mismatch of step " + boost::lexical_cast<std::string>(step.idx) + " Plan: '" + step_plan.header.frame_id + "' vs. step target: '" +
                                       foot_step.target.header.frame_id + "'");
    }

    if (step.idx != step_idx)
      status +=
          ErrorStatusWarning(msgs::ErrorStatus::WARN_INCOMPLETE_STEP_PLAN, "isConsistent",
                             "Wrong step index order: Expected " + boost::lexical_cast<std::string>(step_idx) + " but got " + boost::lexical_cast<std::string>(step.idx) + "!");

    step_idx++;
  }

  return status;
}

std::string ErrorStatusCodeToString(unsigned int error)
{
  switch (error)
  {
    case msgs::ErrorStatus::NO_ERROR:
      return "NO_ERROR";
    case msgs::ErrorStatus::ERR_UNKNOWN:
      return "ERR_UNKNOWN";
    case msgs::ErrorStatus::ERR_NO_SOLUTION:
      return "ERR_NO_SOLUTION";
    case msgs::ErrorStatus::ERR_INVALID_START:
      return "ERR_INVALID_START";
    case msgs::ErrorStatus::ERR_INVALID_GOAL:
      return "ERR_INVALID_GOAL";
    case msgs::ErrorStatus::ERR_INVALID_GRID_MAP:
      return "ERR_INVALID_GRID_MAP";
    case msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL:
      return "ERR_INVALID_TERRAIN_MODEL";
    case msgs::ErrorStatus::ERR_INVALID_STEP:
      return "ERR_INVALID_STEP";
    case msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN:
      return "ERR_INCONSISTENT_STEP_PLAN";
    case msgs::ErrorStatus::ERR_INVALID_PARAMETERS:
      return "ERR_INVALID_PARAMETERS";
    case msgs::ErrorStatus::ERR_NO_PLUGIN_AVAILABLE:
      return "ERR_NO_PLUGIN_AVAILABLE";
    case msgs::ErrorStatus::ERR_INCONSISTENT_REQUEST:
      return "ERR_INCONSISTENT_REQUEST";
    default:
      return "ERR_UNKNOWN";
  }
}

std::string WarningStatusCodeToString(unsigned int warning)
{
  switch (warning)
  {
    case msgs::ErrorStatus::NO_WARNING:
      return "NO_WARNING";
    case msgs::ErrorStatus::WARN_UNKNOWN:
      return "WARN_UNKNOWN";
    case msgs::ErrorStatus::WARN_INCOMPLETE_STEP_PLAN:
      return "WARN_INCOMPLETE_STEP_PLAN";
    case msgs::ErrorStatus::WARN_INVALID_STEP_PLAN:
      return "WARN_INVALID_STEP_PLAN";
    case msgs::ErrorStatus::WARN_NO_TERRAIN_DATA:
      return "WARN_NO_TERRAIN_DATA";
    default:
      return "WARN_UNKNOWN";
  }
}

msgs::ErrorStatus createErrorStatus(const std::string& context, unsigned int error, const std::string& error_msg, unsigned int warning, const std::string& warning_msg, bool output,
                                    double throttle_rate)
{
  if (output)
  {
    if (throttle_rate > 0.0)
    {
      if (error)
        ROS_ERROR_THROTTLE(throttle_rate, "[%s][%s] %s", ErrorStatusCodeToString(error).c_str(), context.c_str(), error_msg.c_str());
      if (warning)
        ROS_WARN_THROTTLE(throttle_rate, "[%s][%s] %s", WarningStatusCodeToString(warning).c_str(), context.c_str(), warning_msg.c_str());
    }
    else
    {
      if (error)
        ROS_ERROR("[%s][%s] %s", ErrorStatusCodeToString(error).c_str(), context.c_str(), error_msg.c_str());
      if (warning)
        ROS_WARN("[%s][%s] %s", WarningStatusCodeToString(warning).c_str(), context.c_str(), warning_msg.c_str());
    }
  }

  msgs::ErrorStatus status;
  status.error = error;
  status.error_msg = error != msgs::ErrorStatus::NO_ERROR ? "[" + ErrorStatusCodeToString(error) + "][" + context + "] " + error_msg : error_msg;
  status.warning = warning;
  status.warning_msg = warning != msgs::ErrorStatus::NO_WARNING ? "[" + WarningStatusCodeToString(warning) + "][" + context + "] " + warning_msg : warning_msg;
  return status;
}

msgs::ErrorStatus ErrorStatusError(unsigned int error, const std::string& context, const std::string& error_msg, bool output, double throttle_rate)
{
  return createErrorStatus(context, error, error_msg, msgs::ErrorStatus::NO_WARNING, "", output, throttle_rate);
}

msgs::ErrorStatus ErrorStatusWarning(unsigned int warning, const std::string& context, const std::string& warning_msg, bool output, double throttle_rate)
{
  return createErrorStatus(context, msgs::ErrorStatus::NO_ERROR, "", warning, warning_msg, output, throttle_rate);
}

bool hasError(const msgs::ErrorStatus& status) { return status.error != msgs::ErrorStatus::NO_ERROR; }
bool hasError(const msgs::ErrorStatus& status, unsigned int code) { return (status.error & code) == code; }

bool hasWarning(const msgs::ErrorStatus& status) { return status.warning != msgs::ErrorStatus::NO_WARNING; }
bool hasWarning(const msgs::ErrorStatus& status, unsigned int code) { return (status.warning & code) == code; }

bool isOk(const msgs::ErrorStatus& status) { return !hasError(status) && !hasWarning(status); }

std::string toString(const msgs::ErrorStatus& error_status)
{
  std::string msg;

  if (error_status.error_msg.size())
    msg = error_status.error_msg;

  if (error_status.warning_msg.size())
  {
    if (msg.size())
      msg += "\n";
    msg = error_status.warning_msg;
  }

  return msg;
}
}  // namespace l3_footstep_planning
