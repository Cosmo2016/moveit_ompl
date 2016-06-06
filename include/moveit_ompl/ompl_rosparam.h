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
  ompl::tools::bolt::SparseGraphPtr sparseGraph = bolt->getSparseGraph();
  ompl::tools::bolt::SparseCriteriaPtr sparseCriteria = bolt->getSparseCriteria();
  ompl::tools::bolt::BoltRetrieveRepairPtr boltRetrieveRepair = bolt->getRetrieveRepairPlanner();
  ompl::tools::bolt::VertexDiscretizerPtr vertexDiscret = sparseCriteria->getVertexDiscretizer();
  ompl::tools::bolt::DenseCachePtr denseCache = sparseGraph->getDenseCache();

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

  // SparseGraph
  {
    ros::NodeHandle rpnh(nh, "sparse_graph");
    error += !get(name, rpnh, "saving_enabled", sparseGraph->savingEnabled_);
    error += !get(name, rpnh, "debug/add_verbose", sparseGraph->vAdd_);
    error += !get(name, rpnh, "visualize/spars_graph", sparseGraph->visualizeSparseGraph_);
    error += !get(name, rpnh, "visualize/spars_graph_speed", sparseGraph->visualizeSparseGraphSpeed_);
    error += !get(name, rpnh, "visualize/database_vertices", sparseGraph->visualizeDatabaseVertices_);
    error += !get(name, rpnh, "visualize/database_edges", sparseGraph->visualizeDatabaseEdges_);
    error += !get(name, rpnh, "visualize/database_coverage", sparseGraph->visualizeDatabaseCoverage_);
    error += !get(name, rpnh, "visualize/quality_path_simp", sparseGraph->visualizeQualityPathSimp_);
    error += !get(name, rpnh, "visualize/astar", sparseGraph->visualizeAstar_);
    error += !get(name, rpnh, "visualize/astar_speed", sparseGraph->visualizeAstarSpeed_);
    shutdownIfError(name, error);
  }

  // SparseCriteria
  {
    ros::NodeHandle rpnh(nh, "sparse_criteria");
    error += !get(name, rpnh, "sparse_delta_fraction", sparseCriteria->sparseDeltaFraction_);
    error += !get(name, rpnh, "dense_delta_fraction", sparseCriteria->denseDeltaFraction_);
    error += !get(name, rpnh, "near_sample_points_multiple", sparseCriteria->nearSamplePointsMultiple_);
    error += !get(name, rpnh, "stretch_factor", sparseCriteria->stretchFactor_);
    error += !get(name, rpnh, "discretize_penetration_dist", sparseCriteria->discretizePenetrationDist_);
    error += !get(name, rpnh, "obstacle_clearance", sparseCriteria->obstacleClearance_);
    error += !get(name, rpnh, "terminate_after_failures", sparseCriteria->terminateAfterFailures_);
    error += !get(name, rpnh, "fourth_criteria_after_failures", sparseCriteria->fourthCriteriaAfterFailures_);
    error += !get(name, rpnh, "sparse_creation_insertion_order", sparseCriteria->sparseCreationInsertionOrder_);
    error += !get(name, rpnh, "percent_max_extent_underestimate", sparseCriteria->percentMaxExtentUnderestimate_);
    error += !get(name, rpnh, "use_discretized_samples", sparseCriteria->useDiscretizedSamples_);
    error += !get(name, rpnh, "use_random_samples", sparseCriteria->useRandomSamples_);
    error += !get(name, rpnh, "use_check_remove_close_vertices", sparseCriteria->useCheckRemoveCloseVertices_);
    error += !get(name, rpnh, "use_clear_edges_near_vertex", sparseCriteria->useClearEdgesNearVertex_);
    error += !get(name, rpnh, "use_original_smoother", sparseCriteria->useOriginalSmoother_);
    error += !get(name, rpnh, "debug/criteria_verbose", sparseCriteria->vCriteria_);
    error += !get(name, rpnh, "debug/quality_verbose", sparseCriteria->vQuality_);
    error += !get(name, rpnh, "debug/remove_close_verbose", sparseCriteria->vRemoveClose_);
    error += !get(name, rpnh, "debug/added_reason_verbose", sparseCriteria->vAddedReason_);
    error += !get(name, rpnh, "visualize/attempted_states", sparseCriteria->visualizeAttemptedStates_);
    error += !get(name, rpnh, "visualize/connectvity", sparseCriteria->visualizeConnectivity_);
    error += !get(name, rpnh, "visualize/remove_close_vertices", sparseCriteria->visualizeRemoveCloseVertices_);
    error += !get(name, rpnh, "visualize/voronoi_diagram", sparseCriteria->visualizeVoronoiDiagram_);
    error += !get(name, rpnh, "visualize/voronoi_diagram_animated", sparseCriteria->visualizeVoronoiDiagramAnimated_);
    error += !get(name, rpnh, "visualize/node_popularity", sparseCriteria->visualizeNodePopularity_);
    error += !get(name, rpnh, "visualize/quality_criteria", sparseCriteria->visualizeQualityCriteria_);
    shutdownIfError(name, error);
  }

  // Dense Cache
  {
    ros::NodeHandle rpnh(nh, "dense_cache");
    error += !get(name, rpnh, "disable_cache", denseCache->disableCache_);
    error += !get(name, rpnh, "enable_cache_saving", denseCache->enableCacheSaving_);
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

    return true;
  }

}  // namespace

#endif  // OMPL_EXPERIENCE_DEMOS_OMPL_ROSPARAM_H_
