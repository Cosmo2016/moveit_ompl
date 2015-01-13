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
   Desc:   
*/

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_HUMANOID_SPACE_HUMANOID_MODEL_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_HUMANOID_SPACE_HUMANOID_MODEL_STATE_SPACE_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

namespace ompl_interface
{

class HumanoidModelStateSpace : public ModelBasedStateSpace
{
public:

  static const std::string PARAMETERIZATION_TYPE;

  HumanoidModelStateSpace(const ModelBasedStateSpaceSpecification &spec);

  /**
   * \brief Custom interpolation function for adjusting a floating virtual joint at the real base to follow a fixed fake base link
   */
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;

  virtual void copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State *state) const;

  virtual void copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const;

protected:

  /** \brief Used to calculate the fake base transform of the vjoint */
  moveit::core::RobotStatePtr moveit_robot_state_;

  /** \brief Used to calculate the fake base transform of the vjoint */
  const moveit::core::JointModel* vjoint_model_;
  int jmg_vjoint_index_;
};

}

#endif
