/**
 * @file fixed_size_interpolation.h
 * @brief
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
CompositeInstruction fixedSizeJointInterpolation(const JointWaypoint& start,
                                                 const JointWaypoint& end,
                                                 const PlanInstruction& base_instruction,
                                                 const PlannerRequest& /*request*/,
                                                 int steps)
{
  CompositeInstruction composite;

  // Linearly interpolate in joint space
  Eigen::MatrixXd states = interpolate(start, end, steps);

  // Convert to MoveInstructions
  for (long i = 1; i < states.cols(); ++i)
  {
    tesseract_planning::MoveInstruction move_instruction(JointWaypoint(states.col(i)), MoveInstructionType::FREESPACE);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }
  return composite;
}

CompositeInstruction fixedSizeJointInterpolation(const JointWaypoint& /*start*/,
                                                 const CartesianWaypoint& /*end*/,
                                                 const PlanInstruction& /*base_instruction*/,
                                                 const PlannerRequest& /*request*/,
                                                 int /*steps*/)
{
  CONSOLE_BRIDGE_logError("fixedSizeLinearInterpolation with Joint/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction fixedSizeJointInterpolation(const CartesianWaypoint& /*start*/,
                                                 const JointWaypoint& /*end*/,
                                                 const PlanInstruction& /*base_instruction*/,
                                                 const PlannerRequest& /*request*/,
                                                 int /*steps*/)
{
  CONSOLE_BRIDGE_logError("fixedSizeLinearInterpolation with Joint/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction fixedSizeJointInterpolation(const CartesianWaypoint& /*start*/,
                                                 const CartesianWaypoint& /*end*/,
                                                 const PlanInstruction& /*base_instruction*/,
                                                 const PlannerRequest& /*request*/,
                                                 int /*steps*/)
{
  CONSOLE_BRIDGE_logError("fixedSizeLinearInterpolation with Joint/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction fixedSizeCartesianInterpolation(const JointWaypoint& start,
                                                     const JointWaypoint& end,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& request,
                                                     int steps)
{
  // Initialize
  auto fwd_kin = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(
      base_instruction.getManipulatorInfo().manipulator);
  auto world_to_base = request.env_state->link_transforms.at(fwd_kin->getBaseLinkName());
  Eigen::Isometry3d tcp = base_instruction.getManipulatorInfo().tcp;

  CompositeInstruction composite;

  // Calculate FK for start and end
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p1, start))
    throw std::runtime_error("fixedSizeLinearInterpolation: failed to find forward kinematics solution!");
  p1 = world_to_base * p1 * tcp;

  Eigen::Isometry3d p2 = Eigen::Isometry3d::Identity();
  if (!fwd_kin->calcFwdKin(p2, end))
    throw std::runtime_error("fixedSizeLinearInterpolation: failed to find forward kinematics solution!");
  p2 = world_to_base * p2 * tcp;

  // Linear interpolation in cartesian space
  tesseract_common::VectorIsometry3d poses = interpolate(p1, p2, steps);

  // Convert to MoveInstructions
  for (std::size_t p = 1; p < poses.size(); ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }

  return composite;
}

CompositeInstruction fixedSizeCartesianInterpolation(const JointWaypoint& /*start*/,
                                                     const CartesianWaypoint& /*end*/,
                                                     const PlanInstruction& /*base_instruction*/,
                                                     const PlannerRequest& /*request*/,
                                                     int /*steps*/)
{
  CONSOLE_BRIDGE_logError("fixedSizeLinearInterpolation with Joint/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction fixedSizeCartesianInterpolation(const CartesianWaypoint& /*start*/,
                                                     const JointWaypoint& /*end*/,
                                                     const PlanInstruction& /*base_instruction*/,
                                                     const PlannerRequest& /*request*/,
                                                     int /*steps*/)
{
  CONSOLE_BRIDGE_logError("fixedSizeLinearInterpolation with Cart/Joint not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction fixedSizeCartesianInterpolation(const CartesianWaypoint& start,
                                                     const CartesianWaypoint& end,
                                                     const PlanInstruction& base_instruction,
                                                     const PlannerRequest& /*request*/,
                                                     int steps)
{
  CompositeInstruction composite;

  // Linear interpolation in cartesian space
  tesseract_common::VectorIsometry3d poses = interpolate(start, end, steps);

  // Convert to MoveInstructions
  for (std::size_t p = 1; p < poses.size(); ++p)
  {
    tesseract_planning::MoveInstruction move_instruction(CartesianWaypoint(poses[p]), MoveInstructionType::LINEAR);
    move_instruction.setManipulatorInfo(base_instruction.getManipulatorInfo());
    move_instruction.setDescription(base_instruction.getDescription());
    composite.push_back(move_instruction);
  }
  return composite;
}

}  // namespace tesseract_planning
