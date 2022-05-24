#include <l3_footstep_planning_libs/modeling/foothold_id.h>

namespace l3_footstep_planning
{
Hash hash_value(const FootholdHashedConstPtrArray& footholds)
{
  // boost::hash_range does use pointer address as hash value
  Hash seed = 0;
  for (FootholdHashed::ConstPtr f : footholds)
    boost::hash_combine(seed, *f);
  return seed;
}
}  // namespace l3_footstep_planning
