#include <tesseract_collision/core/macros.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_PUSH
#include <class_loader/class_loader.h>
TESSERACT_COLLISION_IGNORE_WARNINGS_POP

#include <tesseract_collision/fcl/fcl_discrete_managers.h>

CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_fcl::FCLDiscreteBVHManager, tesseract::DiscreteContactManager)
