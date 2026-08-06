#include "ackermann_ompl_plugins/ackermann_state_space.hpp"

namespace ackermann_ompl_plugins
{

AckermannStateSpace::AckermannStateSpace(double turning_radius)
: ompl::base::ReedsSheppStateSpace(turning_radius)
{
}

}  // namespace ackermann_ompl_plugins
