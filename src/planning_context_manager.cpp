/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <ros/ros.h>

#include <moveit/ompl/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>

// OMPL
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <moveit/ompl/parameterization/joint_space/joint_model_state_space_factory.h>
#include <moveit/ompl/parameterization/work_space/pose_model_state_space_factory.h>
#include <moveit/ompl/parameterization/humanoid_space/humanoid_model_state_space_factory.h>

#include <ompl_thunder/Thunder.h>

// C++
#include <algorithm>
#include <set>

// Boost
#include <boost/filesystem.hpp>

namespace moveit_ompl
{
class PlanningContextManager::LastPlanningContext
{
public:

  ModelBasedPlanningContextPtr getContext()
  {
    boost::mutex::scoped_lock slock(lock_);
    return last_planning_context_solve_;
  }

  void setContext(const ModelBasedPlanningContextPtr &context)
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_ = context;
  }

  void clear()
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_.reset();
  }

private:
  /* The planning group for which solve() was called last */
  ModelBasedPlanningContextPtr last_planning_context_solve_;
  boost::mutex                 lock_;
};

struct PlanningContextManager::CachedContexts
{
  std::map<std::pair<std::string, std::string>,
           std::vector<ModelBasedPlanningContextPtr> > contexts_;
  boost::mutex                                         lock_;
};

} // namespace moveit_ompl

moveit_ompl::PlanningContextManager::PlanningContextManager(const robot_model::RobotModelConstPtr &robot_model, 
                                                            const constraint_samplers::ConstraintSamplerManagerPtr &csm) :
  robot_model_(robot_model), 
  constraint_sampler_manager_(csm),
  max_goal_samples_(10), 
  max_state_sampling_attempts_(1000),  // 4
  max_goal_sampling_attempts_(100), //1000),
  max_planning_threads_(4), 
  max_solution_segment_length_(0.0), 
  minimum_waypoint_count_(5)
{
  last_planning_context_.reset(new LastPlanningContext());
  cached_contexts_.reset(new CachedContexts());
  registerDefaultPlanners();
  registerDefaultStateSpaces();
}

moveit_ompl::PlanningContextManager::~PlanningContextManager()
{
}

namespace
{
using namespace moveit_ompl;

template<typename T>
static ompl::base::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr &si, const std::string &new_name, const ModelBasedPlanningContextSpecification &spec)
{
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  planner->setup();
  return planner;
}
}

moveit_ompl::ConfiguredPlannerAllocator moveit_ompl::PlanningContextManager::plannerSelector(const std::string &planner) const
{
  std::map<std::string, ConfiguredPlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR("Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

void moveit_ompl::PlanningContextManager::registerDefaultPlanners()
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2, _3));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2, _3));
  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2, _3));
}

void moveit_ompl::PlanningContextManager::registerDefaultStateSpaces()
{
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new JointModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new PoseModelStateSpaceFactory()));
  registerStateSpaceFactory(ModelBasedStateSpaceFactoryPtr(new HumanoidModelStateSpaceFactory()));
}

moveit_ompl::ConfiguredPlannerSelector moveit_ompl::PlanningContextManager::getPlannerSelector() const
{
  return boost::bind(&PlanningContextManager::plannerSelector, this, _1);
}

void moveit_ompl::PlanningContextManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
{
  planner_configs_ = pconfig;
}

moveit_ompl::ModelBasedPlanningContextPtr moveit_ompl::PlanningContextManager::getPlanningContext(const std::string &config, const std::string& factory_type, 
                                                                                                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools) const
{
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.find(config);

  if (pc != planner_configs_.end())
  {
    moveit_msgs::MotionPlanRequest req; // dummy request with default values
    return getPlanningContext(pc->second, boost::bind(&PlanningContextManager::getStateSpaceFactory1, this, _1, factory_type), req, visual_tools);
  }
  else
  {
    ROS_ERROR("Planning configuration '%s' was not found", config.c_str());
    return ModelBasedPlanningContextPtr();
  }
}

moveit_ompl::ModelBasedPlanningContextPtr moveit_ompl::PlanningContextManager::getPlanningContext(const planning_interface::PlannerConfigurationSettings &config,
                                                                                                  const StateSpaceFactoryTypeSelector &factory_selector,
                                                                                                  const moveit_msgs::MotionPlanRequest &req,
                                                                                                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools) const
{
  const moveit_ompl::ModelBasedStateSpaceFactoryPtr &factory = factory_selector(config.group);

  // Check for a cached planning context
  ModelBasedPlanningContextPtr context;

  {
    boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> >::const_iterator cc =
      cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));

    // Loop through the cached contextes
    if (cc != cached_contexts_->contexts_.end())
    {
      for (std::size_t i = 0 ; i < cc->second.size() ; ++i)
      {
        //if (cc->second[i].unique()) // check if the context is being shared by anything else
        {
          ROS_DEBUG("Reusing cached planning context");
          context = cc->second[i];
          break;
        }
      }
    }
  }

  // Create a new planning context
  if (!context)
  {
    ModelBasedStateSpaceSpecification space_spec(robot_model_, config.group);
    ModelBasedPlanningContextSpecification context_spec;
    context_spec.config_ = config.config;
    context_spec.planner_selector_ = getPlannerSelector();
    context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
    context_spec.state_space_ = factory->getNewStateSpace(space_spec, visual_tools);


    enum SimpleSetupType
    {
      REGULAR,
      LIGHTNING,
      THUNDER
    };

    // Choose simple setup type
    SimpleSetupType chosen_type;

    // Choose the correct simple setup type to load
    if (req.use_experience) // TODO remove this true hack - allows me to easily benchmarking w and w/o experience
    {
      if (req.num_planning_attempts > 1)
        ROS_ERROR_STREAM_NAMED("planning_context_manager","Number of planning attempts is greater than one, which is not allowed for experienced-based planning. Reducing to 1");

      if (req.experience_method == "lightning")
      {
        chosen_type = LIGHTNING;
      }
      else
      {
        chosen_type = THUNDER; 
      }
    }
    else
    {
      chosen_type = REGULAR;
    }

    // Load correct OMPL setup type
    switch (chosen_type)
    {
      case REGULAR:
        {
          ROS_DEBUG("planning_context_manager: Using regular framework for planning");
          context_spec.ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(context_spec.state_space_));
        }
        break;
      case LIGHTNING:
        {
          ROS_DEBUG("planning_context_manager: Using LIGHTNING Framework for planning");
          context_spec.ompl_simple_setup_.reset(new ompl::tools::Lightning(context_spec.state_space_));

          // Load the experience database
          ompl::tools::Lightning &lightning_handle = static_cast<ompl::tools::Lightning&>(*context_spec.ompl_simple_setup_);

          // Choose the file location
          std::string file_path;
          if (!getFilePath(file_path, context_spec.state_space_->getJointModelGroup()->getName(), "ompl_storage"))
          {
            ROS_ERROR_STREAM_NAMED("planning_context_manager","Unable to find file path for experience framework");
          }

          lightning_handle.setFilePath(file_path);

          if (!req.use_experience)
          {
            ROS_WARN("Lightning Framework is loaded but recall is disabled");
            lightning_handle.enablePlanningFromRecall(false);
          }
        }
        break;
      case THUNDER:
        {
          ROS_DEBUG("planning_context_manager: Using THUNDER Framework for planning");
          context_spec.ompl_simple_setup_.reset(new ompl::tools::Thunder(context_spec.state_space_));

          // Load the experience database
          ompl::tools::Thunder &thunder_handle = static_cast<ompl::tools::Thunder&>(*context_spec.ompl_simple_setup_);

          // Choose the file location
          std::string file_path;
          if (!getFilePath(file_path, context_spec.state_space_->getJointModelGroup()->getName(), "ompl_storage"))
          {
            ROS_ERROR_STREAM_NAMED("planning_context_manager","Unable to find file path for experience framework");
          }

          thunder_handle.setFilePath(file_path);

          // Set other possible parameters
          ros::NodeHandle nh = ros::NodeHandle("~");
          bool use_scratch;
          if (nh.getParam("use_scratch", use_scratch))
          {
            if (!use_scratch)
            {
              ROS_INFO_STREAM_NAMED("planning_context_manager","Planning from scratch disabled via rosparam server");
              thunder_handle.enablePlanningFromScratch(false);
            }
          }
          bool saving_enabled;
          if (nh.getParam("saving_enabled", saving_enabled))
          {
            if (!saving_enabled)
            {
              ROS_INFO_STREAM_NAMED("planning_context_manager","Saving database disabled via rosparam server");
              thunder_handle.getExperienceDB()->setSavingEnabled(false);
            }
          }


          if (!req.use_experience)
          {
            ROS_WARN("Thunder Framework is loaded but recall is disabled");
            thunder_handle.enablePlanningFromRecall(false);
          }
        }
        break;
      default:
        ROS_ERROR("planning_context_manager: No simple setup type found");
    }    

    bool state_validity_cache = true;
    if (config.config.find("subspaces") != config.config.end())
    {
      context_spec.config_.erase("subspaces");
      // if the planner operates at subspace level the cache may be unsafe
      state_validity_cache = false;
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char> > tok(config.config.at("subspaces"), sep);
      for(boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
      {
        const moveit_ompl::ModelBasedStateSpaceFactoryPtr &sub_fact = factory_selector(*beg);
        if (sub_fact)
        {
          ModelBasedStateSpaceSpecification sub_space_spec(robot_model_, *beg);
          context_spec.subspaces_.push_back(sub_fact->getNewStateSpace(sub_space_spec, visual_tools));
        }
      }
    }

    ROS_DEBUG("Creating new planning context");
    context.reset(new ModelBasedPlanningContext(config.name, context_spec, visual_tools));
    context->useStateValidityCache(state_validity_cache);
    {
      boost::mutex::scoped_lock slock(cached_contexts_->lock_);
      cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
    }
  }

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ <= std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(context->getOMPLSimpleSetup()->getStateSpace()->getMaximumExtent() / 100.0);
  else
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);

  context->setSpecificationConfig(config.config);

  last_planning_context_->setContext(context);
  return context;
}

bool moveit_ompl::PlanningContextManager::getFilePath(std::string &file_path, const std::string &database_name, 
                                                      const std::string &database_directory) const
                                                      
{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path rootPath;
  if (!std::string(getenv("HOME")).empty())
    rootPath = fs::path(getenv("HOME")); // Support Linux/Mac
  else if (!std::string(getenv("HOMEPATH")).empty())
    rootPath = fs::path(getenv("HOMEPATH")); // Support Windows
  else
  {
    ROS_WARN("Unable to find a home path for this computer");
    rootPath = fs::path("");
  }

  rootPath = rootPath / fs::path(database_directory);

  boost::system::error_code returnedError;
  fs::create_directories( rootPath, returnedError );

  if ( returnedError )
  {
    //did not successfully create directories
    ROS_ERROR("Unable to create directory %s", database_directory.c_str());
    return false;
  }

  //directories successfully created, append the group name as the file name
  rootPath = rootPath / fs::path(database_name + ".ompl");
  file_path = rootPath.string();
  ROS_INFO_STREAM_NAMED("planning_context_manager","Setting database to " << file_path);

  return true;
}

const moveit_ompl::ModelBasedStateSpaceFactoryPtr& moveit_ompl::PlanningContextManager::getStateSpaceFactory1(const std::string & /* dummy */, const std::string &factory_type) const
{
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator f =
    factory_type.empty() ? state_space_factories_.begin() : state_space_factories_.find(factory_type);
  if (f != state_space_factories_.end())
    return f->second;
  else
  {
    ROS_ERROR("Factory of type '%s' was not found", factory_type.c_str());
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
}

const moveit_ompl::ModelBasedStateSpaceFactoryPtr& moveit_ompl::PlanningContextManager::getStateSpaceFactory2(const std::string &group, const moveit_msgs::MotionPlanRequest &req) const
{
  // find the problem representation to use
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator best = state_space_factories_.end();
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin() ; it != state_space_factories_.end() ; ++it)
  {
    int priority = it->second->canRepresentProblem(group, req, robot_model_);
    if (priority > 0)
      if (best == state_space_factories_.end() || priority > prev_priority)
      {
        best = it;
        prev_priority = priority;
      }
  }

  if (best == state_space_factories_.end())
  {
    ROS_ERROR("There are no known state spaces that can represent the given planning problem");
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
  else
  {
    ROS_DEBUG("Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

moveit_ompl::ModelBasedPlanningContextPtr moveit_ompl::PlanningContextManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                                                                  const moveit_msgs::MotionPlanRequest &req,
                                                                                                  moveit_msgs::MoveItErrorCodes &error_code,
                                                                                                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools) const
{
  // Error check
  if (req.group_name.empty())
  {
    ROS_ERROR("No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ModelBasedPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  // Error check
  if (!planning_scene)
  {
    ROS_ERROR("No planning scene supplied as input");
    return ModelBasedPlanningContextPtr();
  }

  // Identify the correct planning configuration
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ? req.group_name + "[" + req.planner_id + "]" : req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN("Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
              req.group_name.c_str(), req.planner_id.c_str());

  }
  if (pc == planner_configs_.end())
  {
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR("Cannot find planning configuration for group '%s'", req.group_name.c_str());
      return ModelBasedPlanningContextPtr();
    }
  }

  // Choose best planning context
  ModelBasedPlanningContextPtr context = getPlanningContext(pc->second, boost::bind(&PlanningContextManager::getStateSpaceFactory2, this, _1, req), req, visual_tools);
  if (context)
  {
    context->clear();

    robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

    // Setup the context
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    context->setCompleteInitialState(*start_state);

    context->setPlanningVolume(req.workspace_parameters);
    if (!context->setPathConstraints(req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    // Create the goal
    if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
      return ModelBasedPlanningContextPtr();

    try
    {
      context->configure();
      ROS_DEBUG("%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception &ex)
    {
      ROS_ERROR("OMPL encountered an error: %s", ex.what());
      context.reset();
    }
  }

  return context;
}

moveit_ompl::ModelBasedPlanningContextPtr moveit_ompl::PlanningContextManager::getLastPlanningContext() const
{
  return last_planning_context_->getContext();
}
