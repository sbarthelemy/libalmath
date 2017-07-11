include_directories(
 "${CMAKE_CURRENT_SOURCE_DIR}/../../libalmathinternal")
# some tests would require some parts of almathinternal
set(WITH_ALMATHINTERNAL OFF)
if (WITH_ALMATHINTERNAL)
  set(_extra_srcs
    ../../libalmathinternal/almathinternal/interpolations/alinterpolationbezierutils.h
    ../../libalmathinternal/almathinternal/tools/alpolynomialsolver.h
    ../../libalmathinternal/almathinternal/interpolations/bezierkeys.h
    ../../libalmathinternal/almathinternal/interpolations/differentiablefunction.h
    ../../libalmathinternal/src/interpolations/alinterpolationbezierutils.cpp
    ../../libalmathinternal/src/tools/almathinternal.cpp
    ../../libalmathinternal/src/types/alpolar3d.cpp
    ../../libalmathinternal/src/tools/alpolynomialsolver.cpp)
endif()

qi_create_gtest(test_mayaplugin
  SRC
  test_mayaplugin.cpp
  ../src/mayaqianim.cpp
  ../src/mayaqianim.h
  ../src/mayaurdf.cpp
  ../src/mayaurdf.h
  ../src/utils.cpp
  ../src/utils.h
  ${_extra_srcs}
  DEPENDS
  almath maya boost_chrono
  ARGUMENTS "${ALMODELUTILS_PREFIX_PATH}"
  TIMEOUT 100)

file(MAKE_DIRECTORY "${QI_SDK_DIR}/scripts")
foreach(script IN ITEMS
		getTestJulietteUrdfPath
		getAlmodelutilsPrefixPath
		getAlposturePrefixPath)
  configure_file("${script}.in.mel"
                 "${QI_SDK_DIR}/scripts/${script}.mel" @ONLY)
endforeach()
