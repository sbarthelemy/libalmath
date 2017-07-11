cmake_minimum_required(VERSION 2.8.12)
PROJECT(SoftBankRoboticsMayaPlugin)
find_package(qibuild)

enable_testing()

#   Load modules from module directory
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

set(SoftBankRoboticsMayaPlugin_VERSION "0.0.0")
set(SoftBankRoboticsMayaModule_VERSION "0.0.0")

find_package(Maya REQUIRED)

qi_sanitize_compile_flags()

set(sources
  "src/mayaqianim.cpp"
  "src/mayaqianim.h"
  "src/mayaurdf.cpp"
  "src/mayaurdf.h"
  "src/utils.cpp"
  "src/utils.h"
)

qi_create_lib(
  SoftBankRoboticsMayaPlugin
  MODULE
  "src/plugin.cpp"
  "${sources}"
  NO_INSTALL)

set_source_files_properties(
	"src/plugin.cpp" PROPERTIES COMPILE_DEFINITIONS
  "SOFTBANKROBOTICSMAYAPLUGIN_VERSION=\"${SoftBankRoboticsMayaPlugin_VERSION}\"")

install(
  TARGETS SoftBankRoboticsMayaPlugin
  COMPONENT runtime
  DESTINATION plug-ins)

MAYA_PLUGIN(SoftBankRoboticsMayaPlugin)

qi_use_lib(
  SoftBankRoboticsMayaPlugin
  ALMATH
  BOOST
  EIGEN3
  MAYA)

# when running a "maya library application"
# (a standalone binary using the maya library),
# the maya library will initialize the Qt system which wants to locate
# Qt plugins.
# The plugins reside with Maya's installation path.
# Copy Maya's qt.conf alongside the tests. If the "MAYA_LOCATION" variable is
# set, this will ensure the pluings are searched there.
if (EXISTS "${MAYA_LOCATION}/bin/qt.conf")
  file(COPY "${MAYA_LOCATION}/bin/qt.conf"
       DESTINATION "${QI_SDK_DIR}/bin")
endif()

# Maya searchs extension plugins in a sequence of paths which can be changed
# using a file called "Maya.env".
#
# We generate it for developer convenience.
#
# Copy it to ...Users/<username>/Documents/maya/2017/Maya.env
#
# For details see http://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_GUID_FA51BD26_86F3_4F41_9486_2C3CF52B9E17_htm
#
# Note: this Maya.env format is specific to Windows
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/Maya.env"
     "MAYA_PLUG_IN_PATH=${QI_SDK_DIR}/lib;${CMAKE_CURRENT_SOURCE_DIR}/plug-ins\n"
     "MAYA_SCRIPT_PATH=${QI_SDK_DIR}/scripts;${CMAKE_CURRENT_SOURCE_DIR}/scripts")

# description of the end-user Maya module
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/SoftBankRoboticsMayaModule.mod"
	"+ PLATFORM:win64 MAYAVERSION:2017 SoftBankRoboticsMayaModule ${SoftBankRoboticsMayaModule_VERSION} .\n")

set(ALMODELUTILS_PREFIX_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../alrobotmodel/libalmodelutils")
set(ALPOSTURE_PREFIX_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../alrobotmodel/libalposture")

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/SoftBankRoboticsMayaModule.mod"
  COMPONENT runtime
  DESTINATION ".")

if (false)
  # install data which is required by python parts of the plugin.
  # This is a bit of a hack/shortcut.
  qi_install_data(
    test/julietteY20MP201702.urdf
    SUBFOLDER "alrobotmodel/models")

  qi_install_data(
    ${ALMODELUTILS_PREFIX_PATH}/share/alrobotmodel/meshes/juliette/*.dae
    SUBFOLDER "alrobotmodel/meshes/juliette")

  qi_install_data(
    ${ALMODELUTILS_PREFIX_PATH}/share/alrobotmodel/meshes/juliette*.png
    ${ALMODELUTILS_PREFIX_PATH}/share/alrobotmodel/meshes/juliette*.jpg
    SUBFOLDER "alrobotmodel/meshes")
endif()
add_subdirectory(test)
