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

#ifndef MOVEIT_OMPL_PARAMETERIZATION_HUMANOID_SPACE_HUMANOID_MODEL_STATE_SPACE_
#define MOVEIT_OMPL_PARAMETERIZATION_HUMANOID_SPACE_HUMANOID_MODEL_STATE_SPACE_

#include <moveit/ompl/parameterization/model_based_state_space.h>

namespace moveit_ompl
{

class HumanoidModelStateSpace : public ModelBasedStateSpace
{
public:

  class StateType : public ompl::base::State
  {
  public:
    
    enum
      {
        VALIDITY_KNOWN = 1,
        GOAL_DISTANCE_KNOWN = 2,
        VALIDITY_TRUE = 4,
        IS_START_STATE = 8,
        IS_GOAL_STATE = 16
      };
    
    StateType() 
      : ompl::base::State()
      , values(NULL)
      , tag(-1)
      , flags(0)
      , foot_mode(0)
      , distance(0.0)
    {
    }
    
    void markValid(double d)
    {
      distance = d;
      flags |= GOAL_DISTANCE_KNOWN;
      markValid();
    }
    
    void markValid()
    {
      flags |= (VALIDITY_KNOWN | VALIDITY_TRUE);
    }
    
    void markInvalid(double d)
    {
      distance = d;
      flags |= GOAL_DISTANCE_KNOWN;
      markInvalid();
    }

    void markInvalid()
    {
      flags &= ~VALIDITY_TRUE;
      flags |= VALIDITY_KNOWN;
    }

    bool isValidityKnown() const
    {
      return flags & VALIDITY_KNOWN;
    }

    void clearKnownInformation()
    {
      flags = 0;
    }

    bool isMarkedValid() const
    {
      return flags & VALIDITY_TRUE;
    }

    bool isGoalDistanceKnown() const
    {
      return flags & GOAL_DISTANCE_KNOWN;
    }

    bool isStartState() const
    {
      return flags & IS_START_STATE;
    }

    bool isGoalState() const
    {
      return flags & IS_GOAL_STATE;
    }

    bool isInputState() const
    {
      return flags & (IS_START_STATE | IS_GOAL_STATE);
    }

    void markStartState()
    {
      flags |= IS_START_STATE;
    }

    void markGoalState()
    {
      flags |= IS_GOAL_STATE;
    }

    double *values;
    int tag;
    int flags;
    int foot_mode; // enum from moveit::core::FixedLinkModes in robot_state.h
    double distance;
  };

  static const std::string PARAMETERIZATION_TYPE;

  HumanoidModelStateSpace(const ModelBasedStateSpaceSpecification &spec);

  /**
   * \brief Custom interpolation function for adjusting a floating virtual joint at the real base to follow a fixed fake base link
   */
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;

  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;

  virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;

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
