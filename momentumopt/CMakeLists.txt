#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(momentumopt)

# specify the C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# Some additionnal optimization for the code.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops -fPIC -march=native")
#
# Dependencies
#
find_package(pybind11 CONFIG REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# local depends
find_package(mpi_cmake_modules REQUIRED)
find_package(solver REQUIRED)
find_package(solver_lqr REQUIRED)

#
# Build main library and binaries
#

# variable to store list of targets
set(all_targets)

# library source
set(momentumopt_SRC_FILES
    # files for contacts planning
    src/momentumopt/cntopt/ContactState.cpp
    src/momentumopt/cntopt/TerrainDescription.cpp
    src/momentumopt/cntopt/ContactPlanFromFile.cpp
    src/momentumopt/cntopt/ContactPlanInterface.cpp
    # files for dynamics planning
    src/momentumopt/dynopt/DynamicsState.cpp
    src/momentumopt/dynopt/DynamicsFeedback.cpp
    src/momentumopt/dynopt/DynamicsOptimizer.cpp
    # files for kinematics planning
    src/momentumopt/kinopt/KinematicsState.cpp
    src/momentumopt/kinopt/KinematicsInterface.cpp
    src/momentumopt/kinopt/KinematicsOptimizer.cpp
    # files for setting up environments
    src/momentumopt/setting/PlannerSetting.cpp
    # utilities
    src/momentumopt/utilities/OrientationUtils.cpp
    src/momentumopt/utilities/TrajectoryGenerator.cpp)

# Add Library.
add_library(momentumopt SHARED ${momentumopt_SRC_FILES})
list(APPEND all_targets momentumopt)

# Linking.
target_link_libraries(momentumopt solver::solver)
target_link_libraries(momentumopt solver_lqr::solver_lqr)
target_link_libraries(momentumopt yaml_utils::yaml_utils)
target_link_libraries(momentumopt Eigen3::Eigen)

# Properties.
get_filename_component(CONFIG_PATH config ABSOLUTE)
target_compile_definitions(
  momentumopt
  PRIVATE CFG_SRC_PATH="${CONFIG_PATH}/"
  INTERFACE CFG_SRC_PATH="../share/${PROJECT_NAME}/config/")

# Includes. Add the include dependencies
target_include_directories(
  momentumopt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)

#
# Demos. (binaries)
#
add_executable(demo_momentumopt demos/demo_momentumopt.cpp)
target_link_libraries(demo_momentumopt momentumopt)
list(APPEND all_targets demo_momentumopt)

# command to install the library and binaries
install(
  TARGETS ${all_targets}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

# we also need to install the header files
install(DIRECTORY include/ DESTINATION include)

# the config folder
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install the demo python files in bin
install_scripts(
  demos/demo_momentumopt_biped_from_python.py
  demos/demo_momentumopt_solo12_jump.py
  demos/demo_momentumopt_solo_jump_no_lqr.py
  demos/demo_momentumopt_solo_jump.py
  DESTINATION
  bin)

#
# Python bindings
#
set(pymomentumopt_SRC_FILES
    # files for contacts planning
    srcpy/momentumopt/cntopt/PyTerrain.cpp
    srcpy/momentumopt/cntopt/PyContacts.cpp
    # files for dynamics planning
    srcpy/momentumopt/dynopt/PyDynamics.cpp
    # files for kinematics planning
    srcpy/momentumopt/kinopt/PyKinematics.cpp
    # files for setting up environments
    srcpy/momentumopt/PyMomentumopt.cpp
    srcpy/momentumopt/setting/PyParams.cpp
    srcpy/momentumopt/setting/PySetting.cpp)
pybind11_add_module(pymomentum MODULE ${pymomentumopt_SRC_FILES})
target_link_libraries(pymomentum PRIVATE pybind11::module)
target_link_libraries(pymomentum PRIVATE momentumopt)

# install the bindings
#get_python_install_dir(python_install_dir)
set(${python_install_dir} '/usr/local/lib/python3.8')
install(TARGETS pymomentum DESTINATION python3.8/dist-packages)

# install the python package too
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# Tests.
#
include(CTest)

#
# create the cmake package
#
generate_cmake_package()

#
# building documentation
#
add_documentation()
