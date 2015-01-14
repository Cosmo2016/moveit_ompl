/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   This state space is for robots with fixed fake bases such as feet
*/

#include <moveit/ompl/parameterization/humanoid_space/humanoid_model_state_space_factory.h>
#include <moveit/ompl/parameterization/humanoid_space/humanoid_model_state_space.h>

moveit_ompl::HumanoidModelStateSpaceFactory::HumanoidModelStateSpaceFactory() : ModelBasedStateSpaceFactory()
{
  type_ = HumanoidModelStateSpace::PARAMETERIZATION_TYPE;
}

int moveit_ompl::HumanoidModelStateSpaceFactory::canRepresentProblem(const std::string &group,
                                                                        const moveit_msgs::MotionPlanRequest &req,
                                                                        const robot_model::RobotModelConstPtr &kmodel) const
{
  // This state space is for robots with fixed fake bases such as feet
  if (req.start_state.fixed_links.joint_names.size())
  {
    logWarn("Using humanoid model state space");
    return 200;
  }
  else
  {
    logWarn("NOT using humanoid model state space ");
    return 25;
  }

  return 1;
}

moveit_ompl::ModelBasedStateSpacePtr moveit_ompl::HumanoidModelStateSpaceFactory::allocStateSpace(const ModelBasedStateSpaceSpecification &space_spec) const
{
  return ModelBasedStateSpacePtr(new HumanoidModelStateSpace(space_spec));
}
