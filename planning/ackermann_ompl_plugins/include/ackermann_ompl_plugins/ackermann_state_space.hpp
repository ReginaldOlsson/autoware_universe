#pragma once

#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace ackermann_ompl_plugins
{

class AckermannStateSpace : public ompl::base::ReedsSheppStateSpace
{
public:
  explicit AckermannStateSpace(double turning_radius);
};

}  // namespace ackermann_ompl_plugins
