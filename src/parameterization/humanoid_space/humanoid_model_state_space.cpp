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

moveit_ompl::HumanoidModelStateSpace::HumanoidModelStateSpace(const ModelBasedStateSpaceSpecification &spec)
  : ModelBasedStateSpace(spec)
  , is_connected_() // create a matrix that is base 1 (ignore 0) and initialize to 0
{
  setName(getName() + "_" + PARAMETERIZATION_TYPE);

  // Create a robot state to use for updating the virtual joint based on fixed foot location
  moveit_robot_state1_.reset(new moveit::core::RobotState(spec.robot_model_));

  // Load vjoint information for faster fake base transforms
  vjoint_model_ = spec.robot_model_->getRootJoint();
  jmg_vjoint_index_ = spec_.joint_model_group_->getVariableGroupIndex(vjoint_model_->getName());

  // Populate state machine ----------------------------
  // Staying on same state
  is_connected_[1][1] = 1;
  is_connected_[2][2] = 1;
  is_connected_[3][3] = 1;
  is_connected_[4][4] = 1;
  is_connected_[5][5] = 1;
  // Move clockwise on state
  is_connected_[1][2] = 1;
  is_connected_[2][3] = 1;
  is_connected_[4][5] = 1;
  // Move counter-clockwise on state
  is_connected_[5][4] = 1;
  is_connected_[4][3] = 1;
  is_connected_[3][2] = 1;
  is_connected_[2][1] = 1;
}

ompl::base::State* moveit_ompl::HumanoidModelStateSpace::allocState() const
{
  StateType *state = new StateType();
  state->values = new double[variable_count_];
  return state;
}

void moveit_ompl::HumanoidModelStateSpace::freeState(ompl::base::State *state) const
{
  delete[] state->as<StateType>()->values;
  delete state->as<StateType>();
}

void moveit_ompl::HumanoidModelStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
  memcpy(destination->as<StateType>()->values, source->as<StateType>()->values, state_values_size_);
  destination->as<StateType>()->tag = source->as<StateType>()->tag;
  destination->as<StateType>()->flags = source->as<StateType>()->flags;
  destination->as<StateType>()->distance = source->as<StateType>()->distance;

  // Humanoid vars
  destination->as<StateType>()->fixed_link_primary = source->as<StateType>()->fixed_link_primary;
  destination->as<StateType>()->fixed_links = source->as<StateType>()->fixed_links;
  destination->as<StateType>()->fixed_link_stability = source->as<StateType>()->fixed_link_stability;
  destination->as<StateType>()->fixed_link_transforms = source->as<StateType>()->fixed_link_transforms;
}

unsigned int moveit_ompl::HumanoidModelStateSpace::getSerializationLength() const
{
  // Add the number of values, plus 4 ints, plus fixed link transforms 
  // TODO: is it on that the fixed link transforms can be variable??
  logWarn("getSerializationLength may not work properly");
  return state_values_size_ + sizeof(int) * 4;
}

void moveit_ompl::HumanoidModelStateSpace::serialize(void *serialization, const ompl::base::State *state) const
{
  logWarn("serialize may not work properly");

  // Serialize the ints
  *(reinterpret_cast<int*>(serialization))                 = state->as<StateType>()->tag;
  *(reinterpret_cast<int*>(serialization) + sizeof(int)*1) = state->as<StateType>()->fixed_link_primary;
  *(reinterpret_cast<int*>(serialization) + sizeof(int)*2) = state->as<StateType>()->fixed_links;
  *(reinterpret_cast<int*>(serialization) + sizeof(int)*3) = int(state->as<StateType>()->fixed_link_stability);

  // Serialize the double values
  memcpy(reinterpret_cast<char*>(serialization) + sizeof(int)*4, state->as<StateType>()->values, state_values_size_);

  // Serialize the transforms
  //TODO
}

void moveit_ompl::HumanoidModelStateSpace::deserialize(ompl::base::State *state, const void *serialization) const
{
  logWarn("deserialize may not work properly");

  // Deserialize ints
  state->as<StateType>()->tag = 
    *(reinterpret_cast<const int*>(serialization));
  state->as<StateType>()->fixed_link_primary = 
    *(reinterpret_cast<const int*>(serialization) + sizeof(int)*1);
  state->as<StateType>()->fixed_links = 
    *(reinterpret_cast<const int*>(serialization) + sizeof(int)*2);
  state->as<StateType>()->fixed_link_stability = 
    static_cast<moveit::core::FixedLinkStability>(*(reinterpret_cast<const int*>(serialization) + sizeof(int)*3));

  // Deserialize double values
  memcpy(state->as<StateType>()->values, reinterpret_cast<const char*>(serialization) + sizeof(int)*4, state_values_size_);

  // Deserialize the transforms
  // TODO
}

void moveit_ompl::HumanoidModelStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to,
                                                       const double t, ompl::base::State *state) const
{
  std::cout << std::endl;
  std::cout << "moveit_ompl::HumanoidModelStateSpace::interpolate ------------------------------------------------" << std::endl;

  // interpolate in joint space
  ModelBasedStateSpace::interpolate(from, to, t, state);

  // copy the humanoid properties to the interpolated state
  state->as<StateType>()->fixed_link_primary = from->as<StateType>()->fixed_link_primary;
  state->as<StateType>()->fixed_links = from->as<StateType>()->fixed_links;
  state->as<StateType>()->fixed_link_stability = from->as<StateType>()->fixed_link_stability;

  // change the virtual joint to the proper location based on fixed foot position
  if (state->as<StateType>()->fixed_links == 0)
  {
    logError("moveit_ompl::HumanoidModelStateSpace::interpolate() No fixed links in ompl::State");
    return;
  }

  logInform("moveit_ompl::HumanoidModelStateSpace::interpolate() HAS FEET in ompl::State");

  // Convert to moveit::RobotState
  copyToRobotState(*moveit_robot_state1_, state);

  // Update the transforms of the leg and the joint value of the virtual joint
  std::cout << "Fixed primary: " << moveit_robot_state1_->getDynamicRootLink()->getName() << std::endl;

  //moveit_robot_state1_->updateSingleChainDynamicRoot();
  moveit_robot_state1_->updateWithDynamicRoot(); // TODO: is this the fastest method?

  // Copy just the virtual joint back to the ompl "state", since that is the only joint that changed
  copyJointToOMPLState(state, *moveit_robot_state1_, vjoint_model_, jmg_vjoint_index_);

  /*logDebug("VJOINT: %s", vjoint_model_->getName().c_str());
  int length = 7;
  const double* arr = moveit_robot_state1_->getJointPositions(vjoint_model_);
  for (std::size_t i = 0; i < length; ++i)
  {
    std::cout << "vjoint: " << arr[i] << std::endl;
    }*/
}

double moveit_ompl::HumanoidModelStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  // Only for use with bipeds
  assert( moveit_robot_state1_->getRobotModel()->getFixableLinks().size() < 3);

  if (distance_function_)
  {
    logError("Has custom distance function "); // I'm not sure when this would happen
    return distance_function_(state1, state2);
  }

  // Determine what state (mode) we are in the state machine
  moveit_ompl::BipedFootModes s1_biped_mode = getBipedFootMode( state1 );
  moveit_ompl::BipedFootModes s2_biped_mode = getBipedFootMode( state2 );

  std::cout << "BIPED MODES: state1: " << s1_biped_mode << " state2: " << s2_biped_mode << std::endl;

  // Check if the states cannot connect
  if (!is_connected_[ s1_biped_mode, s2_biped_mode])
  {
    logError("Biped states cannot connect. Distance infinity");
    return std::numeric_limits<double>::infinity();
  }

  // Get index of right foot (0 or 1)
  const int& right_foot_index = moveit_robot_state1_->getRobotModel()->getFixableLinkRightIndex();
  assert(right_foot_index == 0 || right_foot_index == 1); // must have a right foot
  
  // Check if moving between s1 and s2
  if ( (s1_biped_mode == LEFT_COM_LEFT_FIXED && s2_biped_mode == LEFT_COM_BOTH_FIXED) || 
       (s1_biped_mode == LEFT_COM_BOTH_FIXED && s2_biped_mode == LEFT_COM_LEFT_FIXED) )
  {
    // Only must have same left foot
    if (equalJoint(state1, state2, int(!right_foot_index), int(!right_foot_index)))
    {
      logInform("Left foot equal, which is all that is required");
      return ModelBasedStateSpace::distance(state1, state2);          
    }

    logError("Left foot NOT equal. Distance infinity");
    return std::numeric_limits<double>::infinity();
  }

  // Check if moving between s4 and s5
  if ( (s1_biped_mode == RIGHT_COM_BOTH_FIXED && s2_biped_mode == RIGHT_COM_RIGHT_FIXED) || 
       (s1_biped_mode == RIGHT_COM_RIGHT_FIXED && s2_biped_mode == RIGHT_COM_BOTH_FIXED) )
  {
    // Only must have same right foot
    if (equalJoint(state1, state2, right_foot_index, right_foot_index))
    {
      logInform("Right foot equal, which is all that is required");
      return ModelBasedStateSpace::distance(state1, state2);          
    }

    logError("Right foot NOT equal. Distance infinity");
    return std::numeric_limits<double>::infinity();
  }

  // Otherwise moving between s2, s3, or s4. Both feet must be equal
  if (equalJoint(state1, state2, right_foot_index, right_foot_index) && 
      equalJoint(state1, state2, !right_foot_index, !right_foot_index))
  {
    logInform("Both feet equal");
    return ModelBasedStateSpace::distance(state1, state2);          
  }

  logError("Both feet NOT equal. Distance infinity");
  return std::numeric_limits<double>::infinity();  
}

moveit_ompl::BipedFootModes moveit_ompl::HumanoidModelStateSpace::getBipedFootMode(const ompl::base::State *state) const
{
  // Primary can only be of value 0 or 1
  assert(state->as<StateType>()->fixed_link_primary < 2);

  // Check if both fixed
  if (moveit::core::RobotState::fixedLinkEnabledHelper(!state->as<StateType>()->fixed_link_primary, // opposite of primary is secondary
                                                       state->as<StateType>()->fixed_links)) // use bitmask
  {
    // Both fixed
    switch( state->as<StateType>()->fixed_link_stability )
    {
      case moveit::core::COM_LEFT_FOOT:
        return LEFT_COM_BOTH_FIXED;
      case moveit::core::COM_BETWEEN_FEET:
        return BOTH_COM_BOTH_FIXED;
      case moveit::core::COM_RIGHT_FOOT:
        return RIGHT_COM_BOTH_FIXED;
      case moveit::core::NOT_STABLE:
        logError("getBipedMode - NOT STABLE");
        return ERROR;
      case moveit::core::UNKNOWN:
      default:
        logError("getBipedMode - NOT UNKNOWN");
        return ERROR;
    }
  }
  else
  {
    // Only 1 foot fixed
    switch( state->as<StateType>()->fixed_link_stability )
    {
      case moveit::core::COM_LEFT_FOOT:
        return LEFT_COM_LEFT_FIXED;
      case moveit::core::COM_RIGHT_FOOT:
        return RIGHT_COM_RIGHT_FIXED;
      case moveit::core::NOT_STABLE:
        logError("getBipedMode - NOT STABLE");
        return ERROR;
      case moveit::core::COM_BETWEEN_FEET:
        logError("getBipedMode - BETWEEN FEET when only 1 foot fixed");
        return ERROR;
      case moveit::core::UNKNOWN:
      default:
        logError("getBipedMode - NOT UNKNOWN");
        return ERROR;
    }
  }
}

bool moveit_ompl::HumanoidModelStateSpace::equalJoint(const ompl::base::State *state1, const ompl::base::State *state2,
                                                      int index_state1, int index_state2) const
{
  // Compare two Affine3d matricies for equality
  for (unsigned int i = 0 ; i < 16; ++i)
  {
    std::cout << "moveit_ompl::HumanoidModelStateSpace::equalJoint - getting variable " << i
              << " state1: " << state1->as<StateType>()->fixed_link_transforms[index_state1].data()[i]
              << " state1: " << state2->as<StateType>()->fixed_link_transforms[index_state2].data()[i] << std::endl;

    // Check if equal
    if ( fabs(state1->as<StateType>()->fixed_link_transforms[index_state1].data()[i] - 
              state2->as<StateType>()->fixed_link_transforms[index_state2].data()[i]) >
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

  return true;
}

void moveit_ompl::HumanoidModelStateSpace::copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State *state) const
{
  rstate.setJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  // Copy humanoid variables
  rstate.setFixedLinkPrimary( state->as<StateType>()->fixed_link_primary );
  rstate.setFixedLinksMode( state->as<StateType>()->fixed_links );
  rstate.setFixedLinkStability( state->as<StateType>()->fixed_link_stability );
  rstate.setFixedLinkTransforms( state->as<StateType>()->fixed_link_transforms );

  // Update transforms in robot state
  if (rstate.dynamicRootEnabled())
    rstate.updateWithDynamicRoot();
  else
    rstate.update();
}

void moveit_ompl::HumanoidModelStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const
{
  rstate.copyJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  // Copy humanoid variables
  state->as<StateType>()->fixed_link_primary = rstate.getFixedLinkPrimary();  
  state->as<StateType>()->fixed_links = rstate.getFixedLinksMode();  
  state->as<StateType>()->fixed_link_stability = rstate.getFixedLinkStability();
  state->as<StateType>()->fixed_link_transforms = rstate.getFixedLinkTransforms();  

  // clear any cached info (such as validity known or not)
  state->as<StateType>()->clearKnownInformation();
}
