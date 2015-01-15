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

  // Testing
  logError("ROOT LINK");
  spec.robot_model_->getRootJoint()->print();

  // Load vjoint information for faster fake base transforms
  static const std::string vjoint_name = "virtual_joint"; // TODO set this dynamically somehow
  vjoint_model_ = spec_.robot_model_->getJointModel(vjoint_name);
  jmg_vjoint_index_ = spec_.joint_model_group_->getVariableGroupIndex(vjoint_name);
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
  const JointModel* fixed_joint = robot_model_->getFootVirtualFloatingJoint();

  // State machine logic
  if ((state1_mode == moveit::core::RIGHT_FOOT_NO_COM || state1_mode == moveit::core::LEFT_FOOT_NO_COM) &&
      (state2_mode == moveit::core::RIGHT_FOOT_NO_COM || state2_mode == moveit::core::LEFT_FOOT_NO_COM))
  {
    // Both in same mode. If their fixed foot is not the same then we know the states CANNOT connect
    std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - both in TRANSITION state" << std::endl;

    // We know the other foot is fixed because COM is outside either foot
    if (!
    const JointModel* free_joint = robot_model_->getFootVirtualFloatingJoint();


    // Compare the joints values of the fixed foot "floating joint"
    if (!equalJoint(state1, state2, fixed_joint))
    {
      // These states are in different discrete steps, so make distance infinity
      logWarn("Returned distance infinity because fixed feet are in different locations");
      return std::numeric_limits<double>::infinity();
    }


  }
  else if (state1_mode == moveit::core::LEFT_FOOT && state2_mode == LEFT_BOTH_FEET ||
           state2_mode == moveit::core::LEFT_FOOT && state1_mode == LEFT_BOTH_FEET)
  {
    // In a transistion state
    std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - left foot based-transition " << std::endl;
    return true;
  }
  else if (state1_mode == moveit::core::RIGHT_FOOT && state2_mode == RIGHT_BOTH_FEET ||
           state2_mode == moveit::core::RIGHT_FOOT && state1_mode == RIGHT_BOTH_FEET)
  {
    // In a transistion state
    std::cout << "moveit_ompl::HumanoidModelStateSpace::distance - right foot based-transition " << std::endl;
    return true;
  }
  else
  {


  }

  return spec_.joint_model_group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);
}

bool moveit_ompl::HumanoidModelStateSpace::equalJoint(const ompl::base::State *state1, const ompl::base::State *state2, 
                                                      const JointModel* joint) const
{
  for (unsigned int i = joint->getFirstVariableIndex() ; i < joint->getVariableCount(); ++i)
  {
    std::cout << "moveit_ompl::HumanoidModelStateSpace::equalJoint - getting variable " << i 
              << " state1: " << state1->as<StateType>()->values[i] 
              << " state2: " << state2->as<StateType>()->values[i] << std::endl;

    // Check if equal
    if (fabs(state1->as<StateType>()->values[i] - state2->as<StateType>()->values[i]) > std::numeric_limits<double>::epsilon())
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
