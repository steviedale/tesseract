/**
 * @file basic_cartesian_example_node.cpp
 * @brief Basic cartesian example node
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_planners_ros_examples/basic_cartesian_example.h>

using namespace tesseract_planners_ros_examples;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_cartesian_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
  const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot description */
  const std::string TRAJOPT_DESCRIPTION_PARAM = "trajopt_description"; /**< Default ROS parameter for trajopt description */

  //bool plotting = true;
  //bool rviz = true;
  int steps = 5;
  std::string method = "json";
  std::string urdf_xml_string, srdf_xml_string;
  std::string trajopt_config = "";

  // Get ROS Parameters
  //  pnh.param("plotting", plotting, plotting);
  //  pnh.param("rviz", rviz, rviz);
  pnh.param<std::string>("method", method, method);
  pnh.param<int>("steps", steps, steps);
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  if (method == "json"){
    nh.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);
  }

  BasicCartesianExample example(steps, method, urdf_xml_string, srdf_xml_string, trajopt_config);
  example.run();
}
