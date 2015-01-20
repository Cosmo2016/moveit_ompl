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


#include <moveit/ompl/parameterization/humanoid_space/humanoid_model_state_space.h>
#include <limits>

const std::string moveit_ompl::HumanoidModelStateSpace::PARAMETERIZATION_TYPE = "HumanoidModel";

moveit_ompl::HumanoidModelStateSpace::HumanoidModelStateSpace(const ModelBasedStateSpaceSpecification &spec) :
  ModelBasedStateSpace(spec)
{
  setName(getName() + "_" + PARAMETERIZATION_TYPE);

  // Create a robot state to use for updating the virtual joint based on fixed foot location
  moveit_robot_state1_.reset(new moveit::core::RobotState(spec.robot_model_));
  moveit_robot_state2_.reset(new moveit::core::RobotState(spec.robot_model_));

  logWarn("moveit_ompl::HumanoidModelStateSpace::HumanoidModelStateSpace() dynamic root enabled"); // do not delete until above fixed

  // Load vjoint information for faster fake base transforms
  vjoint_model_ = spec.robot_model_->getRootJoint();
  jmg_vjoint_index_ = spec_.joint_model_group_->getVariableGroupIndex(vjoint_model_->getName());
}

void moveit_ompl::HumanoidModelStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to,
                                                       const double t, ompl::base::State *state) const
{
  std::cout << std::endl;
  std::cout << "moveit_ompl::HumanoidModelStateSpace::interpolate ------------------------------------------------" << std::endl;

  // interpolate in joint space
  ModelBasedStateSpace::interpolate(from, to, t, state);

  // copy the stability mode bit since the joint model group won't do this
  const moveit::core::FixedLinkModes &mode = static_cast<moveit::core::FixedLinkModes>(from->as<StateType>()->foot_mode);
  std::cout << "moveit_ompl::HumanoidModelStateSpace::interpolate() - copying foot_mode " << mode << " to new interpolated state" << std::endl;
  state->as<StateType>()->foot_mode = mode;

  // change the virtual joint to the proper location based on fixed foot position
  if (mode > moveit::core::NO_FEET)
  {
    logInform("moveit_ompl::HumanoidModelStateSpace::interpolate() HAS FEET in ompl::State");
    moveit_robot_state1_->setFixedFootMode(mode);
    //moveit_robot_state1_->printFixedLinks();

    // Copy all the joints over
    // TODO: only copy joints pertaining to the leg. this is difficult because OMPL state is unknown ordering.
    // Probably not worth the effort
    copyToRobotState(*moveit_robot_state1_, state);

    // Update the transforms of the leg and the joint value of the virtual joint
    std::cout << "Fixed primary: " << moveit_robot_state1_->getDynamicRootLink()->getName() << std::endl;
    //moveit_robot_state1_->updateSingleChainDynamicRoot();
    moveit_robot_state1_->updateWithDynamicRoot(); // TODO: is this the fastest method?

    // Copy just the virtual joint back to the ompl "state", since that is the only joint that changed
    copyJointToOMPLState(state, *moveit_robot_state1_, vjoint_model_, jmg_vjoint_index_);

    logDebug("VJOINT: %s", vjoint_model_->getName().c_str());
    int length = 7;
    const double* arr = moveit_robot_state1_->getJointPositions(vjoint_model_);
    for (std::size_t i = 0; i < length; ++i)
    {
      std::cout << "vjoint: " << arr[i] << std::endl;
    }

  }
  else
    logError("moveit_ompl::HumanoidModelStateSpace::interpolate() NO_FEET in ompl::State");
}

double moveit_ompl::HumanoidModelStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  if (distance_function_)
  {
    logWarn("Has custom distance function "); // I'm not sure when this would happen
    return distance_function_(state1, state2);
  }

  // Deal with mode variable
  const moveit::core::FixedLinkModes& state1_mode = static_cast<moveit::core::FixedLinkModes>(state1->as<StateType>()->foot_mode);
  const moveit::core::FixedLinkModes& state2_mode = static_cast<moveit::core::FixedLinkModes>(state2->as<StateType>()->foot_mode);
  const moveit::core::JointModel* fixed_joint = moveit_robot_state1_->getRobotModel()->getFixedFootVirtualJoint();
  const moveit::core::JointModel* secondary_joint = moveit_robot_state1_->getRobotModel()->getSecondaryFootVirtualJoint();

  // If one state has the COM in one foot, and the other has the COM in the other foot, there is no way the two can connect
  if ((state1_mode == moveit::core::RIGHT_FOOT_COM && state2_mode == moveit::core::LEFT_FOOT_COM) ||
      (state2_mode == moveit::core::RIGHT_FOOT_COM && state1_mode == moveit::core::LEFT_FOOT_COM))
  {
    logWarn("Returned distance infinity because COM is in the opposite foot between states");
    return std::numeric_limits<double>::infinity();
  }

  // Test if fixed foot is equal
  std::cout << "TESTING FIXED FEET " << std::endl;
  bool fixed_foot_equal = equalJoint(state1, state2, fixed_joint, fixed_joint);

  // There are 6 cases where it is ok for the fixed foot to not be equal
  //   1 & 2: when both states are in transition i.e. both have their COM outside a single foot, but the fixed foot is opposite
  //   3 - 6: when one state has the COM in one foot, and in the other state the COM is between feet but the fixed foot is the other one
  if ((state1_mode == moveit::core::RIGHT_FOOT_NO_COM && state2_mode == moveit::core::LEFT_FOOT_NO_COM) ||
      (state2_mode == moveit::core::RIGHT_FOOT_NO_COM && state1_mode == moveit::core::LEFT_FOOT_NO_COM) ||
      // Check for differing fixed foot
      (state1_mode == moveit::core::RIGHT_FOOT_COM && state2_mode == moveit::core::LEFT_FOOT_NO_COM) ||
      (state1_mode == moveit::core::LEFT_FOOT_COM &&  state2_mode == moveit::core::RIGHT_FOOT_NO_COM) ||
      // now try with state2 having the COM in the foot
      (state2_mode == moveit::core::RIGHT_FOOT_COM && state1_mode == moveit::core::LEFT_FOOT_NO_COM) ||
      (state2_mode == moveit::core::LEFT_FOOT_COM &&  state1_mode == moveit::core::RIGHT_FOOT_NO_COM))
  {
    std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - both in TRANSITION state" << std::endl;

    // The fixed feet are switched but it is possible the two states are standing on the same feet positions
    std::cout << "TESTING SWITCHED FEET " << std::endl;
    if (equalJoint(state1, state2, fixed_joint, secondary_joint) &&
        equalJoint(state1, state2, secondary_joint, fixed_joint))
    {
      // They are in fact standing on the same feet. Calculate regular distance
      return ModelBasedStateSpace::distance(state1, state2);
    }
    else
    {
      logWarn("Returned distance infinity because both feet are in diff locations");
      return std::numeric_limits<double>::infinity();
    }
  }

  // Otherwise, check if fixed foot is not equal and if they are not, then distance is infinity
  if (!fixed_foot_equal)
  {
    logWarn("Returned distance infinity because fixed feet are in different locations");
    return std::numeric_limits<double>::infinity();
  }

  // Check when COM is in fixed foot for both states
  if (state1_mode == moveit::core::LEFT_FOOT_COM && state2_mode == moveit::core::LEFT_FOOT_COM ||
      state1_mode == moveit::core::RIGHT_FOOT_COM && state2_mode == moveit::core::RIGHT_FOOT_COM)
  {
    std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - both states have COM in same foot " << std::endl;
    return ModelBasedStateSpace::distance(state1, state2);
  }

  std::cout << "TESTING SECONDARY FOOT " << std::endl;
  bool secondary_foot_equal = equalJoint(state1, state2, secondary_joint, secondary_joint);

  // Check when both are in the same dual foot mode
  if ((state1_mode == moveit::core::RIGHT_FOOT_NO_COM && state2_mode == moveit::core::RIGHT_FOOT_NO_COM) ||
      (state1_mode == moveit::core::LEFT_FOOT_NO_COM  && state2_mode == moveit::core::LEFT_FOOT_NO_COM))
  {
    if (secondary_foot_equal)
    {
      std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - both states have COM between feet, but with same foot locations" << std::endl;
      return ModelBasedStateSpace::distance(state1, state2);
    }
    else
    {
      logWarn("Returned distance infinity because secondary feet are in different locations");
      return std::numeric_limits<double>::infinity();
    }
  }

  // Check if both have same fixed foot but one has COM inside and one has COM between feet
  if ((state1_mode == moveit::core::RIGHT_FOOT_COM && state2_mode == moveit::core::RIGHT_FOOT_NO_COM) ||
      (state1_mode == moveit::core::LEFT_FOOT_COM  && state2_mode == moveit::core::LEFT_FOOT_NO_COM) ||
      (state2_mode == moveit::core::RIGHT_FOOT_COM && state1_mode == moveit::core::RIGHT_FOOT_NO_COM) ||
      (state2_mode == moveit::core::LEFT_FOOT_COM  && state1_mode == moveit::core::LEFT_FOOT_NO_COM))
  {
    if (secondary_foot_equal)
    {
      std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - same fixed foot, one state has COM between feet and the other has COM in one foot" << std::endl;
      return ModelBasedStateSpace::distance(state1, state2);
    }
    else
    {
      logWarn("Returned distance infinity because secondary feet are in different locations");
      return std::numeric_limits<double>::infinity();
    }    
  }

  logError("Unknown foot mode combination between states 1 and 2 - programmer error. This should not happen");
  logError(" state1_mode: %d, state2_mode: %d", state1_mode, state2_mode);
  std::cout << "secondary_foot_equal: " << secondary_foot_equal << std::endl;
  std::cout << "fixed_foot_equal: " << fixed_foot_equal << std::endl;
  assert(false);
  return std::numeric_limits<double>::infinity();
}

bool moveit_ompl::HumanoidModelStateSpace::equalJoint(const ompl::base::State *state1, const ompl::base::State *state2,
                                                      const moveit::core::JointModel* joint1, const moveit::core::JointModel* joint2) const
{
  // Both joints should be poses and contain 7 doubles
  assert(joint1->getVariableCount() == joint2->getVariableCount());

  std::size_t index_state1 = joint1->getFirstVariableIndex();
  std::size_t index_state2 = joint2->getFirstVariableIndex();

  for (unsigned int i = 0 ; i < joint1->getVariableCount(); ++i)
  {
    std::cout << "moveit_ompl::HumanoidModelStateSpace::equalJoint - getting variable " << i
              << " state1: " << state1->as<StateType>()->values[index_state1]
              << " state2: " << state2->as<StateType>()->values[index_state2] << std::endl;

    // Check if equal
    if (fabs(state1->as<StateType>()->values[index_state1] - state2->as<StateType>()->values[index_state2]) >
        std::numeric_limits<double>::epsilon())
    {
      // Not equal
      return false;
    }
  }
  return true;
}

bool moveit_ompl::HumanoidModelStateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  // Compare all the joints values
  for (unsigned int i = 0 ; i < variable_count_; ++i)
    if (fabs(state1->as<StateType>()->values[i] - state2->as<StateType>()->values[i]) > std::numeric_limits<double>::epsilon())
      return false;

  if (true) // TODO remove this
  {
    // Deal with mode variable
    const moveit::core::FixedLinkModes& state1_mode = static_cast<moveit::core::FixedLinkModes>(state1->as<StateType>()->foot_mode);
    const moveit::core::FixedLinkModes& state2_mode = static_cast<moveit::core::FixedLinkModes>(state2->as<StateType>()->foot_mode);

    std::cout << "moveit_ompl::HumanoidModelStateSpace::equalStates -  " << state1_mode << ", " << state2_mode << std::endl;
    if (state1_mode != state2_mode)
    {
      logError("Modes not equal!");
      assert(false);
    }
  }

  return true;
}

void moveit_ompl::HumanoidModelStateSpace::copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State *state) const
{
  rstate.setJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  // Copy the foot mode
  rstate.setFixedFootMode( static_cast<moveit::core::FixedLinkModes>(state->as<StateType>()->foot_mode ));

  std::cout << "moveit_ompl::HumanoidModelStateSpace::copyToRobotState foot_mode = " << state->as<StateType>()->foot_mode << std::endl;

  if (rstate.dynamicRootEnabled())
  {
    rstate.updateDynamicRootLink(); // convert the double values from OMPL to a correct Transform structure
    rstate.updateWithDynamicRoot(); // TODO: is this the fastest method?
  }
  else
    rstate.update();
}

void moveit_ompl::HumanoidModelStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const
{
  rstate.copyJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  // Copy the foot mode
  state->as<StateType>()->foot_mode = rstate.getFixedFootMode();
  std::cout << "moveit_ompl::HumanoidModelStateSpace::copyToOMPLState last_value = " << state->as<StateType>()->foot_mode << std::endl;

  // clear any cached info (such as validity known or not)
  state->as<StateType>()->clearKnownInformation();
}
