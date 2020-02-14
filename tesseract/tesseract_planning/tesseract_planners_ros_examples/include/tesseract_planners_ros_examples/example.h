/**
 * @file examples.h
 * @brief Examples base class
 *
 * @author Levi Armstrong
 * @date July 22, 2018
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
#ifndef TESSERACT_ROS_EXAMPLES_EXAMPLES_H
#define TESSERACT_ROS_EXAMPLES_EXAMPLES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <ros/console.h>
#include <ros/service_client.h>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

namespace tesseract_planners_ros_examples
{
/**
 * @brief The Example base class
 *
 * It provides a generic interface for all examples as a library which then
 * can easily be integrated as unit tests so breaking changes are caught.
 * Also it provides a few utility functions for checking rviz environment and
 * updating the rviz environment.
 */
class Example
{
public:
  Example(std::string urdf_xml_string, std::string srdf_xml_string, std::string trajopt_config="")
    : urdf_xml_string_(urdf_xml_string), srdf_xml_string_(srdf_xml_string), trajopt_config_(trajopt_config), tesseract_(std::make_shared<tesseract::Tesseract>())
  {
  }
  virtual ~Example() = default;
  Example(const Example&) = default;
  Example& operator=(const Example&) = default;
  Example(Example&&) = default;
  Example& operator=(Example&&) = default;

  virtual bool run() = 0;

protected:
  tesseract::Tesseract::Ptr tesseract_;     /**< @brief Tesseract Manager Class */
  std::string urdf_xml_string_;
  std::string srdf_xml_string_;
  std::string trajopt_config_;
};
}  // namespace tesseract_planners_ros_examples
#endif  // TESSERACT_ROS_EXAMPLES_EXAMPLES_H
