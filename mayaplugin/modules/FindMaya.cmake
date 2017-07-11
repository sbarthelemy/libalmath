if (NOT DEFINED MAYA_VERSION)
	set(MAYA_VERSION 2017 CACHE STRING "Maya version")
endif()

###################
#  Default value  #
###################

set(MAYA_COMPILE_DEFINITIONS "REQUIRE_IOSTREAM;_BOOL")
set(MAYA_INSTALL_BASE_SUFFIX "")
set(MAYA_LIB_SUFFIX "lib")
set(MAYA_INC_SUFFIX "include")

if (WIN32)
	
	###########
	# Windows #
	###########

	set(MAYA_INSTALL_BASE_DEFAULT "C:/Program Files/Autodesk")
	set(OPENMAYA OpenMaya.lib)
	set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};NT_PLUGIN")
	set(MAYA_PLUGIN_EXTENSION ".mll")

elseif(APPLE)
	
	#########
	#  MAC  #
	#########

	set(MAYA_INSTALL_BASE_DEFAULT "/Application/Autodesk")
	set(OPENMAYA libOpenMaya.dylib)
	set(MAYA_LIB_SUFFIX "Maya.app/Content/MacOS")
	set(MAYA_INC_SUFFIX "devkit/include")
	set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};OSMac")
	set(MAYA_PLUGIN_EXTENSION ".bundle")

else()

	#########
	# Linux #
	#########

	set(MAYA_INSTALL_BASE_DEFAULT "/usr/autodesk")
	set(MAYA_INSTALL_BASE_SUFFIX -x64)
	set(OPENMAYA libOpenMaya.so)
	set(MAYA_COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS};LINUX")
	set(MAYA_PLUGIN_EXTENSION ".so")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

endif()

##########################
#  Path to Maya location #
##########################

set(MAYA_INSTALL_BASE_PATH ${MAYA_INSTALL_BASE_DEFAULT} CACHE STRING "Root Maya installation path")
set(MAYA_LOCATION ${MAYA_INSTALL_BASE_PATH}/maya${MAYA_VERSION}${MAYA_INSTALL_BASE_SUFFIX})

find_path(MAYA_LIBRARY_DIR ${OPENMAYA}
	PATHS
		${MAYA_LOCATION}
		$ENV{MAYA_LOCATION}
	PATH_SUFFIXES
		"${MAYA_LIB_SUFFIX}/"
	DOC "Maya library path"
)

find_path(MAYA_INCLUDE_DIRS maya/
	PATHS
		${MAYA_LOCATION}
		$ENV{MAYA_VERSION}
	PATH_SUFFIXES
		"${MAYA_INC_SUFFIX}/"
	DOC "Maya include path"
)


###############
#  Used libs  #
###############

set(_MAYA_LIBRARIES Foundation OpenMaya OpenMayaUI OpenMayaAnim OpenMayaFX OpenMayaRender)
foreach(MAYA_LIB ${_MAYA_LIBRARIES})
	find_library(MAYA_${MAYA_LIB}_LIBRARY NAMES ${MAYA_LIB} PATHS ${MAYA_LIBRARY_DIR} NO_DEFAULT_PATH)
	list(APPEND MAYA_LIBRARIES ${MAYA_${MAYA_LIB}_LIBRARY})
endforeach()


set(MAYA_INCLUDE_DIRS
  "${MAYA_INCLUDE_DIRS}"
  CACHE INTERNAL "" FORCE
)

set(MAYA_LIBRARIES
  "${MAYA_LIBRARIES}"
  CACHE INTERNAL "" FORCE
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Maya DEFAULT_MSG MAYA_INCLUDE_DIRS MAYA_LIBRARIES)

export_lib(MAYA)

#######################
#  flags compilation  #
#######################

function(MAYA_PLUGIN _target)
	if (WIN32)
		set_target_properties(${_target} PROPERTIES
			LINK_FLAGS "/export:initializePlugin /export:uninitializePlugin")
	endif()
	set_target_properties(${_target} PROPERTIES 
		COMPILE_DEFINITIONS "${MAYA_COMPILE_DEFINITIONS}"
		PREFIX ""
		SUFFIX ${MAYA_PLUGIN_EXTENSION})
endfunction()
