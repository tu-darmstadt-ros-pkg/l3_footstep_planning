#include <l3_footstep_planning_libs/modeling/floating_base_id.h>

namespace l3_footstep_planning
{
Hash hash_value(const FloatingBaseHashedConstPtrArray& bases)
{
  // boost::hash_range does use pointer address as hash value
  Hash seed = 0;
  for (FloatingBaseHashed::ConstPtr f : bases)
    boost::hash_combine(seed, *f);
  return seed;
}
}  // namespace l3_footstep_planning
