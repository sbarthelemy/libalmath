#ifndef SOFTBANKROBOTICSMAYAPLUGIN_SRC_UTILS_H
#define SOFTBANKROBOTICSMAYAPLUGIN_SRC_UTILS_H

#include <maya/MPlug.h>
#include <maya/MStatus.h>
#include <maya/MDagPath.h>
#include <maya/MLibrary.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MFnTransform.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MFnNumericAttribute.h>

#include <utility>
#include <map>
#include <sstream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/math/constants/constants.hpp>
#include <almath/scenegraph/urdf.h>

using boost::property_tree::ptree;

// poor man's error reporting
// inspired from CHECK_MSTATUS_AND_RETURN_IT
#define CHECK_MSTATUS_AND_THROW_IT(_status)         \
if (true) {                                         \
  MStatus _maya_status = (_status);                 \
  if ( MStatus::kSuccess != _maya_status ) {        \
    cerr << "\nAPI error detected in " << __FILE__  \
         << " at line " << __LINE__ << endl;        \
    _maya_status.perror ( "" );                     \
    throw (_status);                                \
    }                                               \
}

namespace SBRMP {

// create a fstream from a filename given as a MString.
//
// we might run into problems on Windows when opening files with extended
// characters in their filename.
// These helper should enbale us to centralize the fix.
std::fstream create_fstream(
  const MString &filename,
  std::ios_base::openmode mode = std::ios_base::in | std::ios_base::out);

std::ifstream create_ifstream(
  const MString &filename,
  std::ios_base::openmode mode = std::ios_base::in);

std::ofstream create_ofstream(
  const MString &filename,
  std::ios_base::openmode mode = std::ios_base::out);

// change Maya's current namespace in the ctor and restore the previous one
// in the dtor.
// The namespace is created if needed.
// If newNamespace is empty, this is a no-op
//
// Note: ideally, we should provide a way to undo it. We don't
class ScopedCurrentNamespace{
public:
  ScopedCurrentNamespace(const MString &newNamespace);
  ~ScopedCurrentNamespace();
private:
  MString previousNamespace;
};

// Throw if the given fps cannot be represented as a Maya time unit.
MTime::Unit unitFromFps(int fps);

// Throw if the given unit cannot be represented as an integer number
// of frames per second.
int fpsFromUnit(MTime::Unit unit);

enum struct Axis { x, y, z };

enum struct ScrewAxis { rx, ry, rz, tx, ty, tz };

inline ScrewAxis rotationDof(Axis axis) {
  switch (axis) {
  case Axis::x: return ScrewAxis::rx;
  case Axis::y: return ScrewAxis::ry;
  case Axis::z: return ScrewAxis::rz;
  }
  throw std::invalid_argument("invalid axis");
};

inline ScrewAxis translationDof(Axis axis) {
  switch (axis) {
  case Axis::x: return ScrewAxis::tx;
  case Axis::y: return ScrewAxis::ty;
  case Axis::z: return ScrewAxis::tz;
  }
  throw std::invalid_argument("invalid axis");
};

inline Axis toAxis(ScrewAxis screwaxis) {
  switch (screwaxis) {
  case ScrewAxis::rx: return Axis::x;
  case ScrewAxis::tx: return Axis::x;
  case ScrewAxis::ry: return Axis::y;
  case ScrewAxis::ty: return Axis::y;
  case ScrewAxis::rz: return Axis::z;
  case ScrewAxis::tz: return Axis::z;
  }
  throw std::invalid_argument("invalid screwAxis");
};

std::pair<MFnTransform::LimitType, MFnTransform::LimitType>
toMayaTransformLimitsType(ScrewAxis axis);

MString toMayaTransformAttrName(ScrewAxis axis);
ScrewAxis screwAxisFromMayaTransformAttrName(MString name);

MStatus lockIkJointPlugs(MFnIkJoint &ikJointFn);
MStatus lockTransformPlugs(MFnTransform &transformFn);

MStatus unlockTransformPlug(MFnTransform &transformFn, ScrewAxis dof);

MStatus setLimits(MFnIkJoint &ikJointFn, ScrewAxis axis,
  const std::pair<double, double> &limits);

template <typename T>
MStatus setAttr(MObject depNodeObj, const char* attr, T value) {
  MStatus status = MStatus::kFailure;
  MFnDependencyNode depNodeFn(depNodeObj);
  if (depNodeFn.hasAttribute(attr)) {
    MPlug plug = depNodeFn.findPlug(attr, true, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plug.setValue(value);
    CHECK_MSTATUS_AND_RETURN_IT(status);
  }
  return status;
}

std::vector<MPlug> getIkJointAnimatablePlugs(const MObject &ikJointObj);

typedef std::multimap<std::string, MPlug> PlugsMap;

PlugsMap getIkJointsAnimatablePlugsMap(MObject root = MObject::kNullObj);

// return the time of the last key from any AnimCurve connected to a plug
MTime maxAnimCurveTime(const PlugsMap &plugsMap);

MStatus importColladaMesh(MString filename, MString groupname, MObject &object);

MObject addNurbsCircle(Axis normalaxis,
                       double radius,
                       MObject parent = MObject::kNullObj);

void addControllers(const PlugsMap &plugsMap, double radius);
}

#endif