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
  ompl::tools::bolt::DenseDBPtr denseDB = bolt->getDenseDB();
  ompl::tools::bolt::DiscretizerPtr discretizer = denseDB->getDiscretizer();
  ompl::tools::bolt::SparseDBPtr sparseDB = denseDB->getSparseDB();
  ompl::tools::bolt::BoltRetrieveRepairPtr boltRetrieveRepair = bolt->getRetrieveRepairPlanner();

  // Bolt
  {
    ros::NodeHandle rpnh(nh, "bolt");
    error += !get(name, rpnh, "visualize/raw_trajectory", bolt->visualizeRawTrajectory_);
  }

  // Discretizer
  {
    ros::NodeHandle rpnh(nh, "discretizer");
    error += !get(name, rpnh, "discretization", discretizer->discretization_);
    error += !get(name, rpnh, "visualize/grid_generation", discretizer->visualizeGridGeneration_);
  }

  // DenseDB
  {
    ros::NodeHandle rpnh(nh, "dense_db");
    error += !get(name, rpnh, "desired_average_cost", denseDB->desiredAverageCost_);
    error += !get(name, rpnh, "debug/snap_path_verbose", denseDB->snapPathVerbose_);
    error += !get(name, rpnh, "visualize/cart_neighbors", denseDB->visualizeCartNeighbors_);
    error += !get(name, rpnh, "visualize/cart_path", denseDB->visualizeCartPath_);
    error += !get(name, rpnh, "visualize/snap_path", denseDB->visualizeSnapPath_);
    error += !get(name, rpnh, "visualize/snap_path_speed", denseDB->visualizeSnapPathSpeed_);
    error += !get(name, rpnh, "visualize/astar", denseDB->visualizeAstar_);
    error += !get(name, rpnh, "visualize/astar_speed", denseDB->visualizeAstarSpeed_);
    error += !get(name, rpnh, "visualize/add_sample", denseDB->visualizeAddSample_);
    error += !get(name, rpnh, "visualize/database_edges", denseDB->visualizeDatabaseEdges_);
    error += !get(name, rpnh, "visualize/database_vertices", denseDB->visualizeDatabaseVertices_);
    error += !get(name, rpnh, "save_database", denseDB->savingEnabled_);
    error += !get(name, rpnh, "popularity_bias_enabled", denseDB->popularityBiasEnabled_);
    error += !get(name, rpnh, "popularity_bias", denseDB->popularityBias_);
    shutdownIfError(name, error);
  }

  // SparseDB
  {
    ros::NodeHandle rpnh(nh, "sparse_db");
    error += !get(name, rpnh, "sparse_delta_fraction", sparseDB->sparseDeltaFraction_);
    sparseDB->discretization_ = discretizer->discretization_;
    error += !get(name, rpnh, "dense_delta_fraction", sparseDB->denseDeltaFraction_);
    error += !get(name, rpnh, "near_sample_points_multiple", sparseDB->nearSamplePointsMultiple_);
    error += !get(name, rpnh, "stretch_factor", sparseDB->stretchFactor_);
    error += !get(name, rpnh, "sparse_creation_insertion_order", sparseDB->sparseCreationInsertionOrder_);
    error += !get(name, rpnh, "percent_max_extent_underestimate", sparseDB->percentMaxExtentUnderestimate_);
    error += !get(name, rpnh, "testing_bool", sparseDB->testingBool_);
    error += !get(name, rpnh, "debug/checks_verbose", sparseDB->checksVerbose_);
    error += !get(name, rpnh, "visualize/spars_graph", sparseDB->visualizeSparsGraph_);
    error += !get(name, rpnh, "visualize/spars_graph_speed", sparseDB->visualizeSparsGraphSpeed_);
    error += !get(name, rpnh, "visualize/database_vertices", sparseDB->visualizeDatabaseVertices_);
    error += !get(name, rpnh, "visualize/database_edges", sparseDB->visualizeDatabaseEdges_);
    error += !get(name, rpnh, "visualize/dense_representatives", sparseDB->visualizeDenseRepresentatives_);
    error += !get(name, rpnh, "visualize/node_popularity", sparseDB->visualizeNodePopularity_);
    error += !get(name, rpnh, "visualize/quality_criteria", sparseDB->visualizeQualityCriteria_);
    shutdownIfError(name, error);
  }

  // BoltRetrieveRepair
  {
    ros::NodeHandle rpnh(nh, "bolt_retrieve_repair");
    error += !get(name, rpnh, "debug/verbose", boltRetrieveRepair->verbose_);
    error += !get(name, rpnh, "visualize/astar", boltRetrieveRepair->visualizeAstar_);
    error += !get(name, rpnh, "visualize/astar_speed", boltRetrieveRepair->visualizeAstarSpeed_);
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
