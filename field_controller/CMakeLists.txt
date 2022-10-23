list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/bme688_module
)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(field_controller)

set_property(TARGET app PROPERTY CXX_STANDARD 11)

target_compile_definitions(app PRIVATE
  EIDSP_USE_CMSIS_DSP=1
  EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
  ARM_MATH_LOOPUNROLL
)

include(edge-impulse-sdk/cmake/utils.cmake)
add_subdirectory(edge-impulse-sdk/cmake/zephyr)

target_include_directories(app PRIVATE
  .
  include
  tflite-model
  model-parameters
)

RECURSIVE_FIND_FILE(CPP_SOURCE_FILES "src" "*.cpp")
RECURSIVE_FIND_FILE(MODEL_FILES "tflite-model" "*.cpp")
list(APPEND SOURCE_FILES ${CPP_SOURCE_FILES} ${MODEL_FILES})

target_sources(app PRIVATE ${SOURCE_FILES})
target_sources(app PRIVATE src/main.cpp)