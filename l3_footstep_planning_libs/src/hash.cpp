#include <l3_footstep_planning_libs/hash.h>

namespace l3
{
using namespace l3_footstep_planning;

Hash hash_value(const Foothold& foothold)
{
  Hash seed = 0;
  boost::hash_combine(seed, foothold.idx);
  boost::hash_combine(seed, foothold.x());
  boost::hash_combine(seed, foothold.y());
  boost::hash_combine(seed, foothold.z());
  boost::hash_combine(seed, foothold.yaw());
  return seed;
}

Hash hash_value(const FootholdArray& array)
{
  Hash seed = 0;
  for (const Foothold& f : array)
    boost::hash_combine(seed, f);
  return seed;
}
}  // namespace l3
