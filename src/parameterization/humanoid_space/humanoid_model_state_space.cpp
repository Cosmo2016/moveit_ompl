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


#include <moveit/ompl_interface/parameterization/humanoid_space/humanoid_model_state_space.h>

const std::string ompl_interface::HumanoidModelStateSpace::PARAMETERIZATION_TYPE = "HumanoidModel";

ompl_interface::HumanoidModelStateSpace::HumanoidModelStateSpace(const ModelBasedStateSpaceSpecification &spec) :
  ModelBasedStateSpace(spec)
{
  setName(getName() + "_" + PARAMETERIZATION_TYPE);

  // Create a robot state to use for updating the virtual joint based on fixed foot location
  moveit_robot_state_.reset(new moveit::core::RobotState(spec.robot_model_));

  if (true) // TODO check this somehow moveit_robot_state_->dynamicRootEnabled())
  {
    logWarn("ompl_interface::HumanoidModelStateSpace::HumanoidModelStateSpace() dynamic root enabled"); // do not delete until above fixed

    // Load vjoint information for faster fake base transforms
    static const std::string vjoint_name = "virtual_joint"; // TODO set this dynamically somehow
    vjoint_model_ = spec_.robot_model_->getJointModel(vjoint_name);
    jmg_vjoint_index_ = spec_.joint_model_group_->getVariableGroupIndex(vjoint_name);
  }
  else
  {
    logError("ompl_interface::HumanoidModelStateSpace::HumanoidModelStateSpace() dynamic root not enabled");
  }
}

void ompl_interface::HumanoidModelStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to,
                                                          const double t, ompl::base::State *state) const
{
  std::cout << "ompl_interface::HumanoidModelStateSpace::interpolate" << std::endl;

  // interpolate in joint space
  ModelBasedStateSpace::interpolate(from, to, t, state);

  // copy the stability mode bit since the joint model group won't do this
  const moveit::core::FixedLinkModes &mode 
    = static_cast<moveit::core::FixedLinkModes>(from->as<StateType>()->values[variable_count_ - 1]);
  std::cout << "ompl_interface::HumanoidModelStateSpace::interpolate() - copying over mode " << mode << std::endl;
  state->as<StateType>()->values[variable_count_ - 1] = mode;

  // change the virtual joint to the proper location based on fixed foot position
  if (mode > moveit::core::NO_FEET)
  {    
    logInform("ompl_interface::HumanoidModelStateSpace::interpolate() HAS FEET in ompl::State");
    moveit_robot_state_->setFixedFootMode(mode);
    moveit_robot_state_->printFixedLinks();

    // Copy all the joints over
    // TODO: only copy joints pertaining to the leg. this is difficult because OMPL state is unknown ordering. 
    // Probably not worth the effort
    copyToRobotState(*moveit_robot_state_, state);

    // Update the transforms of the leg and the joint value of the virtual joint
    std::cout << "Fixed primary: " << moveit_robot_state_->getDynamicRootLink()->getName() << std::endl;
    //moveit_robot_state_->updateSingleChainDynamicRoot();
    moveit_robot_state_->updateWithDynamicRoot(); // TODO: is this the fastest method?

    // Copy just the virtual joint back to the ompl "state", since that is the only joint that changed
    copyJointToOMPLState(state, *moveit_robot_state_, vjoint_model_, jmg_vjoint_index_);

    // TODO: set this just once at construction, somehow
    moveit_robot_state_->enableDynamicRoot();
  }
  else
    logError("ompl_interface::HumanoidModelStateSpace::interpolate() NO_FEET in ompl::State");
}

void ompl_interface::HumanoidModelStateSpace::copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State *state) const
{
  rstate.setJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  // Get the foot mode from the last position of the data
  rstate.setFixedFootMode( static_cast<moveit::core::FixedLinkModes>(state->as<StateType>()->values[variable_count_ - 1]) );

  std::cout << "copyToRobotState last_value = " << state->as<StateType>()->values[variable_count_ - 1] << std::endl;
  
  if (rstate.dynamicRootEnabled())
  {
    logWarn("calling updateWithDynamicRoot()");
    rstate.updateWithDynamicRoot(); // TODO: is this the fastest method?
  }
  else
    rstate.update();

  std::cout << std::endl;
}

void ompl_interface::HumanoidModelStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const
{
  rstate.copyJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  // The foot mode goes in the last position of the data
  state->as<StateType>()->values[variable_count_ - 1] = double(int(rstate.getFixedFootMode()));
  std::cout << "copyToOMPLState last_value = " << state->as<StateType>()->values[variable_count_ - 1] << std::endl;

  // clear any cached info (such as validity known or not)
  state->as<StateType>()->clearKnownInformation();
}
