## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

cmake_minimum_required(VERSION 2.6.4)
project(ALMATH)
find_package(qibuild)

enable_testing()

# disable boost circular buffer "debug helper"
# note that by default the helper is:
# * disabled in release build
# * enabled on debug build for boost < 1.62
# * disabled on debug build for boost >= 1.62
# see also:
#  https://svn.boost.org/trac/boost/ticket/6277
#  https://redmine.aldebaran.lan/issues/19739
add_definitions("-DBOOST_CB_DISABLE_DEBUG")

option(ALMATH_WITH_QIGEOMETRY
  "if true, qigeometry and geometry_module are built"
  ON)
option(ALMATH_AS_STATIC_LIBRARY
  "if true, almath is built as a static library"
  OFF)
option(ALMATH_WITH_MAYAPLUGIN
  "if true, the maya plugin is built"
  OFF)

if (ALMATH_WITH_QIGEOMETRY)
  find_package(qilang-tools)

  # generate all the new files
  qi_gen_idl(geometry_idl CPP "qigeometry" "${CMAKE_CURRENT_BINARY_DIR}"
    qi/geometry/geometry.idl.qi)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  # install the headers so that people can use your proxies
  qi_install_header(
    ${geometry_idl_INTERFACE}
    qi/geometry/api.hpp
    SUBFOLDER qi/geometry)
  # create a lib with the proxies only
  qi_create_lib(qigeometry
    ${geometry_idl_INTERFACE}
    ${geometry_idl_LOCAL}
    ${geometry_idl_REMOTE}
    DEPENDS QI)
  qi_stage_lib(qigeometry)
endif()

# Add BEFORE keyword so that we can rebuild almath against a sdk
# without getting conflicts.
include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}")

set(ALMATH_SRCS
    src/dsp/digitalfilter.cpp
    src/dsp/pidcontroller.cpp
    src/geometrics/shapes3d.cpp
    src/geometrics/shapes3d_utils.cpp
    src/scenegraph/colladabuilder.cpp
    src/scenegraph/colladascenebuilder.cpp
    src/scenegraph/mesh.cpp
    src/scenegraph/meshfactory.cpp
    src/scenegraph/qianim.cpp
    src/scenegraph/qianim/surgeon.cpp
    src/scenegraph/scenebuilder.cpp
    src/scenegraph/urdf.cpp
    src/scenegraph/urdfrigidbodysystemtraverser.cpp
    src/tools/avoidfootcollision.cpp
    src/tools/almath.cpp
    src/tools/almathio.cpp
    src/tools/aldubinscurve.cpp
    src/tools/altransformhelpers.cpp
    src/types/alaxismask.cpp
    src/types/alpose2d.cpp
    src/types/alrotation3d.cpp
    src/types/alrotation.cpp
    src/types/alpositionandvelocity.cpp
    src/types/altransformandvelocity6d.cpp
    src/types/altransform.cpp
    src/types/alvelocity3d.cpp
    src/types/alvelocity6d.cpp
    src/types/alposition2d.cpp
    src/types/alposition3d.cpp
    src/types/alposition6d.cpp
    src/types/alquaternion.cpp
    src/types/aldisplacement.cpp
    src/types/occupancymapparams.cpp
)

# list of the public headers which are wrapped
# check it against swig -MM almath.i
set(ALMATH_H_WRAPPED
    almath/dsp/digitalfilter.h
    almath/dsp/pidcontroller.h
    almath/types/alaxismask.h
    almath/types/alpose2d.h
    almath/types/alposition2d.h
    almath/types/alposition3d.h
    almath/types/alposition6d.h
    almath/types/alpositionandvelocity.h
    almath/types/alquaternion.h
    almath/types/aldisplacement.h
    almath/types/alrotation.h
    almath/types/alrotation3d.h
    almath/types/altransform.h
    almath/types/alvelocity3d.h
    almath/types/alvelocity6d.h
    almath/types/altransformandvelocity6d.h
    almath/tools/aldubinscurve.h
    almath/tools/almath.h
    almath/tools/altransformhelpers.h
    almath/tools/altrigonometry.h
    almath/tools/avoidfootcollision.h
    almath/types/occupancymapparams.h
)
set(ALMATH_H
    ${ALMATH_H_WRAPPED}
    almath/geometrics/shapes3d.h
    almath/geometrics/shapes3d_utils.h
    almath/scenegraph/almatheigen.h
    almath/scenegraph/almathqigeometry.hpp
    almath/scenegraph/qianim.h
    almath/scenegraph/qianim/bezierautotangent.h
    almath/scenegraph/qianim/unit.h
    almath/scenegraph/qianim/unitutils.h
    almath/scenegraph/qianim/surgeon.h
    almath/scenegraph/bodymass.h
    almath/scenegraph/colladabuilder.h
    almath/scenegraph/colladascenebuilder.h
    almath/scenegraph/gltf.h
    almath/scenegraph/meshfactory.h
    almath/scenegraph/mesh.h
    almath/scenegraph/qigeometry.h
    almath/scenegraph/rigidbodysystembuilder.h
    almath/scenegraph/qirostime.h
    almath/scenegraph/scenebuilder.h
    almath/scenegraph/urdf.h
    almath/scenegraph/urdfeigen.h
    almath/scenegraph/urdfrigidbodysystemtraverser.h
    almath/tools/almathio.h
)

if(ALMATH_AS_STATIC_LIBRARY)
  qi_create_lib(almath STATIC ${ALMATH_SRCS} ${ALMATH_H})
else()
  qi_create_lib(almath SHARED ${ALMATH_SRCS} ${ALMATH_H})
  set_target_properties(almath PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()

qi_use_lib(almath BOOST EIGEN3)

# generate a header that will define the symbol visibility/deprecation macros:
#  ALMATH_API
#  ALMATH_DEPRECATED
# tweak it so that it behaves like QI_LIB_API from qi/macro.hpp
include(GenerateExportHeader)
generate_export_header(almath
  EXPORT_MACRO_NAME ALMATH_API
  STATIC_DEFINE almath_STATIC_BUILD
  EXPORT_FILE_NAME ${QI_SDK_DIR}/${QI_SDK_INCLUDE}/almath/api.h)
qi_install_header(${QI_SDK_DIR}/${QI_SDK_INCLUDE}/almath/api.h SUBFOLDER almath)

# ensure generated headers are found
include_directories(${QI_SDK_DIR}/${QI_SDK_INCLUDE})

# we publish SWIG_MODULE_almathswig_DEPS together with the lib.
# It is useful when extending almathswig.
set(SWIG_MODULE_almathswig_DEPS "almath.i")
foreach(dep IN LISTS ALMATH_H_WRAPPED)
  list(APPEND SWIG_MODULE_almathswig_DEPS "${dep}")
endforeach()
qi_stage_lib(almath ALMATH
  CUSTOM_CODE
  "set(SWIG_MODULE_almathswig_DEPS \"${SWIG_MODULE_almathswig_DEPS}\" CACHE INTERNAL \"\" FORCE)")
qi_install_header(${ALMATH_H} KEEP_RELATIVE_PATHS)

qi_add_optional_package(PYTHON)
if(WITH_PYTHON)
  include(qibuild/swig/python)
  set(SWIG_MODULE_almathswig_EXTRA_DEPS ${ALMATH_H_WRAPPED})
  qi_swig_wrap_python(almathswig almath.i DEPENDS ALMATH)
  # we install the swig interface file since it is needed to wrap extensions
  # of almath, such as almathinternal.
  qi_install_header(almath.i KEEP_RELATIVE_PATHS)
  qi_install_python(almath.py)

  # Hack for the python interpreter to find the python module even at
  # build time, using qi::path::librariesPath().
  add_custom_target(copy-python-almath ALL
    COMMAND ${CMAKE_COMMAND}
    -E copy
    "${CMAKE_SOURCE_DIR}/almath.py"
    "${QI_SDK_DIR}/${QI_SDK_LIB}/almath.py"
  )
endif()

qi_create_bin(
  animationsurgeon
  bin/animationsurgeon.cpp
  DEPENDS ALMATH BOOST BOOST_PROGRAM_OPTIONS BOOST_FILESYSTEM
  NO_INSTALL)

add_subdirectory(test)

if(ALMATH_WITH_QIGEOMETRY)
  add_subdirectory(geometry_module)
endif()
if(ALMATH_WITH_MAYAPLUGIN)
  add_subdirectory(mayaplugin)
endif()
