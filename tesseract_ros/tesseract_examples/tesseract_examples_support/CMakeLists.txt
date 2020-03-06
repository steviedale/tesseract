cmake_minimum_required(VERSION 3.5.0)
project(tesseract_examples_support VERSION 0.1.0)

foreach(dir urdf meshes config)
   install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()
