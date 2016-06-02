/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helper function for loading rosparameter settings into the
           OMPL Bolt algorithm from different applications
*/

#ifndef OMPL_EXPERIENCE_DEMOS_OMPL_ROSPARAM_H_
#define OMPL_EXPERIENCE_DEMOS_OMPL_ROSPARAM_H_

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// OMPL
#include <ompl/tools/bolt/Bolt.h>

// Boost
#include <boost/filesystem.hpp>

namespace ompl_experience_demos
{
void loadOMPLParameters(ros::NodeHandle nh, const std::string &name, ompl::tools::bolt::BoltPtr bolt)
{
  using namespace rosparam_shortcuts;
  std::size_t error = 0;
  ompl::tools::bolt::SparseDBPtr sparseDB = bolt->getSparseDB();
  ompl::tools::bolt::BoltRetrieveRepairPtr boltRetrieveRepair = bolt->getRetrieveRepairPlanner();
  ompl::tools::bolt::VertexDiscretizerPtr vertexDiscret = sparseDB->getVertexDiscretizer();
  ompl::tools::bolt::DenseCachePtr denseCache = sparseDB->getDenseCache();

  // Bolt
  {
    ros::NodeHandle rpnh(nh, "bolt");
    error += !get(name, rpnh, "visualize/raw_trajectory", bolt->visualizeRawTrajectory_);
  }

  // Vertex Discretizer
  {
    ros::NodeHandle rpnh(nh, "vertex_discretizer");
    error += !get(name, rpnh, "debug/verbose", vertexDiscret->verbose_);
    error += !get(name, rpnh, "visualize/grid_generation", vertexDiscret->visualizeGridGeneration_);
  }

  // SparseDB
  {
    ros::NodeHandle rpnh(nh, "sparse_db");
    error += !get(name, rpnh, "sparse_delta_fraction", sparseDB->sparseDeltaFraction_);
    error += !get(name, rpnh, "dense_delta_fraction", sparseDB->denseDeltaFraction_);
    error += !get(name, rpnh, "near_sample_points_multiple", sparseDB->nearSamplePointsMultiple_);
    error += !get(name, rpnh, "stretch_factor", sparseDB->stretchFactor_);
    error += !get(name, rpnh, "obstacle_clearance", sparseDB->obstacleClearance_);
    error += !get(name, rpnh, "terminate_after_failures", sparseDB->terminateAfterFailures_);
    error += !get(name, rpnh, "fourth_criteria_after_failures", sparseDB->fourthCriteriaAfterFailures_);
    error += !get(name, rpnh, "sparse_creation_insertion_order", sparseDB->sparseCreationInsertionOrder_);
    error += !get(name, rpnh, "percent_max_extent_underestimate", sparseDB->percentMaxExtentUnderestimate_);
    error += !get(name, rpnh, "use_discretized_samples", sparseDB->useDiscretizedSamples_);
    error += !get(name, rpnh, "use_random_samples", sparseDB->useRandomSamples_);
    error += !get(name, rpnh, "use_check_remove_close_vertices", sparseDB->useCheckRemoveCloseVertices_);
    error += !get(name, rpnh, "use_clear_edges_near_vertex", sparseDB->useClearEdgesNearVertex_);
    error += !get(name, rpnh, "use_original_smoother", sparseDB->useOriginalSmoother_);
    error += !get(name, rpnh, "debug/add_verbose", sparseDB->vAdd_);
    error += !get(name, rpnh, "debug/checks_verbose", sparseDB->vCriteria_);
    error += !get(name, rpnh, "debug/quality_verbose", sparseDB->vQuality_);
    error += !get(name, rpnh, "debug/remove_close_verbose", sparseDB->vRemoveClose_);
    error += !get(name, rpnh, "debug/added_reason_verbose", sparseDB->vAddedReason_);
    error += !get(name, rpnh, "visualize/spars_graph", sparseDB->visualizeSparsGraph_);
    error += !get(name, rpnh, "visualize/spars_graph_speed", sparseDB->visualizeSparsGraphSpeed_);
    error += !get(name, rpnh, "visualize/attempted_states", sparseDB->visualizeAttemptedStates_);
    error += !get(name, rpnh, "visualize/connectvity", sparseDB->visualizeConnectivity_);
    error += !get(name, rpnh, "visualize/database_vertices", sparseDB->visualizeDatabaseVertices_);
    error += !get(name, rpnh, "visualize/database_edges", sparseDB->visualizeDatabaseEdges_);
    error += !get(name, rpnh, "visualize/database_coverage", sparseDB->visualizeDatabaseCoverage_);
    error += !get(name, rpnh, "visualize/voronoi_diagram", sparseDB->visualizeVoronoiDiagram_);
    error += !get(name, rpnh, "visualize/voronoi_diagram_animated", sparseDB->visualizeVoronoiDiagramAnimated_);
    error += !get(name, rpnh, "visualize/node_popularity", sparseDB->visualizeNodePopularity_);
    error += !get(name, rpnh, "visualize/quality_criteria", sparseDB->visualizeQualityCriteria_);
    error += !get(name, rpnh, "visualize/quality_path_simp", sparseDB->visualizeQualityPathSimp_);
    error += !get(name, rpnh, "visualize/remove_close_vertices", sparseDB->visualizeRemoveCloseVertices_);
    error += !get(name, rpnh, "visualize/astar", sparseDB->visualizeAstar_);
    error += !get(name, rpnh, "visualize/astar_speed", sparseDB->visualizeAstarSpeed_);
    shutdownIfError(name, error);
  }

  // Dense Cache
  {
    ros::NodeHandle rpnh(nh, "dense_cache");
    error += !get(name, rpnh, "disable_cache", denseCache->disableCache_);
    error += !get(name, rpnh, "save_every_n_edges", denseCache->saveEveryNEdges_);
    shutdownIfError(name, error);
  }

  // BoltRetrieveRepair
  {
    ros::NodeHandle rpnh(nh, "bolt_retrieve_repair");
    error += !get(name, rpnh, "debug/verbose", boltRetrieveRepair->verbose_);
    shutdownIfError(name, error);
  }
}

  /**
   * \brief Creates a directory names *database_direction* in the user's *home* folder, and inside that creates a file
   *        named *database_name.ompl*
   * \param file_path - result to generate
   * \param file_name - name of file to create
   * \param home_directory - name of folder to save in user directory
   * \return true on success
   */
  bool getFilePath(std::string &file_path, const std::string &file_name, const std::string &home_directory)
  {
    namespace fs = boost::filesystem;
    // Check that the directory exists, if not, create it
    fs::path rootPath;

    //rootPath = fs::path("/home/dave");
    //std::cout << "Over-rode root path to : " << rootPath << std::endl;

    if (!std::string(getenv("HOME")).empty())
      rootPath = fs::path(getenv("HOME"));  // Support Linux/Mac
    else if (!std::string(getenv("HOMEPATH")).empty())
      rootPath = fs::path(getenv("HOMEPATH"));  // Support Windows
    else
    {
      ROS_WARN("Unable to find a home path for this computer");
      rootPath = fs::path("");
    }
    rootPath = rootPath / fs::path(home_directory);

    boost::system::error_code returnedError;
    fs::create_directories(rootPath, returnedError);

    if (returnedError)
    {
      // did not successfully create directories
      ROS_ERROR("Unable to create directory %s", home_directory.c_str());
      return false;
    }

    // directories successfully created, append the group name as the file name
    rootPath = rootPath / fs::path(file_name);
    file_path = rootPath.string();
    ROS_INFO_STREAM("Loading from " << file_path);

    return true;
  }

}  // namespace

#endif  // OMPL_EXPERIENCE_DEMOS_OMPL_ROSPARAM_H_
