#include "utils.h"
#include <maya/MFileIO.h>
#include <maya/MGlobal.h>
#include <maya/MNamespace.h>
#include <maya/MItDag.h>
#include <maya/MPointArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnNurbsCurve.h>

#include <fstream>

namespace SBRMP {

// Doc about MString and encoding
// http://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_GUID_258601E3_3783_4709_8DD7_916BD60FD918_htm
std::fstream create_fstream(const MString &filename,
                            std::ios_base::openmode mode) {
  // Note: maybe on Visual Studio we should instead do
  // return std::fstream(filename.asWChar(), mode)
  return std::fstream(filename.asChar(), mode);
}

std::ifstream create_ifstream(const MString &filename,
                              std::ios_base::openmode mode) {
  // Note: maybe on Visual Studio we should instead do
  // return std::fstream(filename.asWChar(), mode)
  return std::ifstream(filename.asChar(), mode);
}

std::ofstream create_ofstream(const MString &filename,
                              std::ios_base::openmode mode) {
  // Note: maybe on Visual Studio we should instead do
  // return std::ofstream(filename.asWChar(), mode)
  return std::ofstream(filename.asChar(), mode);
}

MTime::Unit unitFromFps(int fps) {
  switch (fps)
  {
  case 2:
    return MTime::Unit::k2FPS;
  case 3:
    return MTime::Unit::k3FPS;
  case 4:
    return MTime::Unit::k4FPS;
  case 5:
    return MTime::Unit::k5FPS;
  case 6:
    return MTime::Unit::k6FPS;
  case 8:
    return MTime::Unit::k8FPS;
  case 10:
    return MTime::Unit::k10FPS;
  case 12:
    return MTime::Unit::k12FPS;
  case 15:
    return MTime::Unit::kGames;
  case 16:
    return MTime::Unit::k16FPS;
  case 20:
    return MTime::Unit::k20FPS;
  case 24:
    return MTime::Unit::kFilm;
  case 25:
    return MTime::Unit::kPALFrame;
  case 30:
    return MTime::Unit::kNTSCFrame;
  case 40:
    return MTime::Unit::k40FPS;
  case 48:
    return MTime::Unit::kShowScan;
  case 50:
    return MTime::Unit::kPALField;
  case 60:
    return MTime::Unit::kNTSCField;
  case 75:
    return MTime::Unit::k75FPS;
  case 80:
    return MTime::Unit::k80FPS;
  case 100:
    return MTime::Unit::k100FPS;
  case 120:
    return MTime::Unit::k120FPS;
  case 125:
    return MTime::Unit::k125FPS;
  case 150:
    return MTime::Unit::k150FPS;
  case 200:
    return MTime::Unit::k200FPS;
  case 240:
    return MTime::Unit::k240FPS;
  case 250:
    return MTime::Unit::k250FPS;
  case 300:
    return MTime::Unit::k300FPS;
  case 375:
    return MTime::Unit::k375FPS;
  case 400:
    return MTime::Unit::k400FPS;
  case 500:
    return MTime::Unit::k500FPS;
  case 600:
    return MTime::Unit::k600FPS;
  case 750:
    return MTime::Unit::k750FPS;
  case 1200:
    return MTime::Unit::k1200FPS;
  case 1500:
    return MTime::Unit::k1500FPS;
  case 2000:
    return MTime::Unit::k2000FPS;
  case 3000:
    return MTime::Unit::k3000FPS;
  case 6000:
    return MTime::Unit::k6000FPS;
  }
  std::stringstream msg;
  msg << "no MTime::Unit can represent " << fps << " frames per second.";
  throw(std::invalid_argument(msg.str()));
}

int fpsFromUnit(MTime::Unit unit) {
  switch (unit)
  {
  case MTime::Unit::k2FPS:
    return 2;
  case MTime::Unit::k3FPS:
    return 3;
  case MTime::Unit::k4FPS:
    return 4;
  case MTime::Unit::k5FPS:
    return 5;
  case MTime::Unit::k6FPS:
    return 6;
  case MTime::Unit::k8FPS:
    return 8;
  case MTime::Unit::k10FPS:
    return 10;
  case MTime::Unit::k12FPS:
    return 12;
  case MTime::Unit::kGames:
    return 15;
  case MTime::Unit::k16FPS:
    return 16;
  case MTime::Unit::k20FPS:
    return 20;
  case MTime::Unit::kFilm:
    return 24;
  case MTime::Unit::kPALFrame:
    return 25;
  case MTime::Unit::kNTSCFrame:
    return 30;
  case MTime::Unit::k40FPS:
    return 40;
  case MTime::Unit::kShowScan:
    return 48;
  case MTime::Unit::kPALField:
    return 50;
  case MTime::Unit::kNTSCField:
    return 60;
  case MTime::Unit::k75FPS:
    return 75;
  case MTime::Unit::k80FPS:
    return 80;
  case MTime::Unit::k100FPS:
    return 100;
  case MTime::Unit::k120FPS:
    return 120;
  case MTime::Unit::k125FPS:
    return 125;
  case MTime::Unit::k150FPS:
    return 150;
  case MTime::Unit::k200FPS:
    return 200;
  case MTime::Unit::k240FPS:
    return 240;
  case MTime::Unit::k250FPS:
    return 250;
  case MTime::Unit::k300FPS:
    return 300;
  case MTime::Unit::k375FPS:
    return 375;
  case MTime::Unit::k400FPS:
    return 400;
  case MTime::Unit::k500FPS:
    return 500;
  case MTime::Unit::k600FPS:
    return 600;
  case MTime::Unit::k750FPS:
    return 750;
  case MTime::Unit::k1200FPS:
    return 1200;
  case MTime::Unit::k1500FPS:
    return 1500;
  case MTime::Unit::k2000FPS:
    return 2000;
  case MTime::Unit::k3000FPS:
    return 3000;
  case MTime::Unit::k6000FPS:
    return 6000;
  case MTime::Unit::kInvalid:
  case MTime::Unit::kUserDef:
  case MTime::Unit::kLast:
  case MTime::Unit::kHours:
  case MTime::Unit::kMinutes:
    break;
  }
  throw std::invalid_argument(
    "this MTime::Unit cannot be represented as an integer number of frames"
    " per second");
}

std::pair<MFnTransform::LimitType, MFnTransform::LimitType>
toMayaTransformLimitsType(ScrewAxis axis) {
  using R = std::pair<MFnTransform::LimitType, MFnTransform::LimitType>;
  switch (axis) {
  case ScrewAxis::rx:
    return R(MFnTransform::kRotateMinX, MFnTransform::kRotateMaxX);
  case ScrewAxis::ry:
    return R(MFnTransform::kRotateMinY, MFnTransform::kRotateMaxY);
  case ScrewAxis::rz:
    return R(MFnTransform::kRotateMinZ, MFnTransform::kRotateMaxZ);
  case ScrewAxis::tx:
    return R(MFnTransform::kTranslateMinX, MFnTransform::kTranslateMaxX);
  case ScrewAxis::ty:
return R(MFnTransform::kTranslateMinY, MFnTransform::kTranslateMaxY);
  case ScrewAxis::tz:
    return R(MFnTransform::kTranslateMinZ, MFnTransform::kTranslateMaxZ);
  }
  throw std::invalid_argument("input matched no MFnTransform::LimitType pair");
}

MString toMayaTransformAttrName(ScrewAxis axis) {
  switch (axis) {
  case ScrewAxis::rx:
    return "rx";
  case ScrewAxis::ry:
    return "ry";
  case ScrewAxis::rz:
    return "rz";
  case ScrewAxis::tx:
    return "tx";
  case ScrewAxis::ty:
    return "ty";
  case ScrewAxis::tz:
    return "tz";
  }
  throw std::invalid_argument("input matched no MFnTransform attribute");
}

ScrewAxis screwAxisFromMayaTransformAttrName(MString name) {
  if (name == "rx")
    return ScrewAxis::rx;
  if (name == "ry")
    return ScrewAxis::ry;
  if (name == "rz")
    return ScrewAxis::rz;
  if (name == "tx")
    return ScrewAxis::tx;
  if (name == "ty")
    return ScrewAxis::ty;
  if (name == "tz")
    return ScrewAxis::tz;
  throw std::invalid_argument("input matched no ScrewAxis");
}

MStatus lockTransformPlugs(MFnTransform &transformFn) {
  if (transformFn.object().isNull()) {
    return MStatus::kFailure;
  }
  MStatus status;
  const auto isFree = false;
  for (auto attrName : {
    "rx", "ry", "rz", // rotate
    "tx", "ty", "tz", // translate
    "sx", "sy", "sz", // scale
    "rax", "ray", "raz", // rotate axis
    "shearXY", "shearXZ", "shearYZ" }) {
    auto plug = transformFn.findPlug(attrName, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plug.setKeyable(isFree);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plug.setLocked(!isFree);
    CHECK_MSTATUS_AND_RETURN_IT(status);
  }
  return status;
}

MStatus lockIkJointPlugs(MFnIkJoint &ikJointFn) {
  if (ikJointFn.object().isNull()) {
    return MStatus::kFailure;
  }
  MStatus status = lockTransformPlugs(ikJointFn);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  const auto isFree = false;
  for (auto attrName : { "jointOrientX", "jointOrientY", "jointOrientZ" }) {
    auto plug = ikJointFn.findPlug(attrName, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plug.setKeyable(isFree);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plug.setLocked(!isFree);
    CHECK_MSTATUS_AND_RETURN_IT(status);
  }
  return status;
}

MStatus unlockTransformPlug(MFnTransform &transformFn, ScrewAxis dof) {
  MStatus status;
  const auto attrName = toMayaTransformAttrName(dof);
  auto plug = transformFn.findPlug(attrName, false, &status);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plug.setKeyable(true);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = plug.setLocked(false);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  return status;
}

std::vector<MPlug> getIkJointAnimatablePlugs(const MObject &ikJointObj) {
  MStatus status;
  std::vector<MPlug> plugs;
  MFnDependencyNode ikJointFn(ikJointObj);
  for (auto attrName : {
    "rx", "ry", "rz", // rotate
    "tx", "ty", "tz" }) { // translate
    MPlug plug = ikJointFn.findPlug(attrName, false, &status);
    if (status != MStatus::kSuccess)
      continue;
    bool isKeyable = plug.isKeyable(&status);
    if ((status == MStatus::kSuccess) && isKeyable)
      plugs.push_back(plug);
  }
  return plugs;
}

MStatus setLimits(MFnIkJoint &ikJointFn, ScrewAxis axis,
  const std::pair<double, double> &limits) {
  const auto limitsId = toMayaTransformLimitsType(axis);
  MStatus status;

  status = ikJointFn.setLimit(limitsId.first, limits.first);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = ikJointFn.enableLimit(limitsId.first, true);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  status = ikJointFn.setLimit(limitsId.second, limits.second);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = ikJointFn.enableLimit(limitsId.second, true);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  return status;
}

ScopedCurrentNamespace::ScopedCurrentNamespace(const MString &newNamespace) {
  if (newNamespace.length() == 0u)
    return;
  MStatus status;
  previousNamespace = MNamespace::currentNamespace(&status);
  // Create the namespace.
  // Will fail if the namespace already exists.
  // Note that we cannot use
  //     MNamespace::namespaceExists(newNamespace, &status);
  // because it does not account for relative namespaces.
  status = MNamespace::addNamespace(newNamespace);
  //CHECK_MSTATUS(status);
  status = MNamespace::setCurrentNamespace(newNamespace);
  CHECK_MSTATUS(status);
}

ScopedCurrentNamespace::~ScopedCurrentNamespace() {
  if (previousNamespace.length() == 0u)
    return;
  auto status = MNamespace::setCurrentNamespace(previousNamespace);
  CHECK_MSTATUS(status);
}

MStatus importColladaMesh(MString filename, MString groupname,
                          MObject &object) {
  MStatus status;
  if (false) {
    MFileIO fileio;
    // TODO: visual_namespace argument is ignored
    // see http://forums.autodesk.com/t5/maya-programming/importing-a-collada-mesh-from-c-an-alternative-to-mfileio/m-p/6830714
    MString nnamespace("_visual");
    status = fileio.importFile(filename, "DAE_FBX", false, nnamespace.asChar());
    CHECK_MSTATUS_AND_RETURN_IT(status);
  }
  else {
    // first import the file as children of world.
    // Since we use the -groupName options, they will be ancestors of
    // one pkTranskorm node
    //
    // beware, this part is vulnerable to MEL injection
    auto cmd = MString("file -returnNewNodes -groupReference -groupName \"") +
      groupname + "\" -import -type \"DAE_FBX\" \""
      + filename + "\";";
    MStringArray names;
    status = MGlobal::executeCommand(cmd, names);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // Iterate over the world kTransform children, in order to find the
    // "-groupName" one. We compare with `names`
    // instead of `groupname` in case maya changed it (namespacing or so).
    // Since we iterate breadth-first, we'll find the good one.
    MItDag dagIter(MItDag::kBreadthFirst, MFn::kTransform);
    while (!dagIter.isDone() && object.isNull()) {
      auto fullPathName = dagIter.fullPathName(&status);
      CHECK_MSTATUS_AND_RETURN_IT(status);
      for (auto i = 0u; i < names.length(); ++i) {
        if (names[i] == fullPathName) {
          object = dagIter.currentItem(&status);
          CHECK_MSTATUS_AND_RETURN_IT(status);
          break;
        }
      }
      dagIter.next();
    }
    if (object.isNull()) {
      std::cerr << "failed to find the node imported from \""
        << filename << "\"";
      return MStatus::kFailure;
    }
  }
  return status;
}

std::multimap<std::string, MPlug> getIkJointsAnimatablePlugsMap(
    const MObject root) {
  std::multimap<std::string, MPlug> ret;
  MStatus status;
  MItDag dagIter(MItDag::kDepthFirst, MFn::kJoint);

  if (root != MObject::kNullObj) {
    status = dagIter.reset(root, MItDag::kDepthFirst, MFn::kJoint);
    CHECK_MSTATUS_AND_THROW_IT(status);
  }
  MFnIkJoint ikJointFn;
  while (!dagIter.isDone()) {
    ikJointFn.setObject(dagIter.currentItem());
    for (const auto &p : getIkJointAnimatablePlugs(dagIter.currentItem())) {
      ret.emplace(ikJointFn.name().asChar(), p);
    }
    dagIter.next();
  }
  return ret;
}

MTime maxAnimCurveTime(const PlugsMap &plugsMap) {
  MTime res(0.);
  MStatus status;
  for (const auto &kv : plugsMap) {
    const auto& plug = kv.second;
    if (plug.isNull() || !plug.isConnected())
      continue;
    MFnAnimCurve animCurveFn(plug, &status);
    CHECK_MSTATUS_AND_THROW_IT(status);
    auto k = animCurveFn.numKeys();
    if (k > 0u)
      res = std::max(res, animCurveFn.time(k - 1u));
  }
  return res;
}

MObject addNurbsCircle(Axis normalAxis, double radius, MObject parent) {
  MStatus status = MStatus::kFailure;
  MFnNurbsCurve circleFn;
  const int ncvs = 50;
  const int nknots = (ncvs - 3) + 2 * 3 - 1;
  MDoubleArray knotSequences;
  MPointArray controlVerticles;

  MPoint p;
  for (auto i = 0; i < ncvs; i++) {
    switch (normalAxis)
    {
    case Axis::x:
      p.x = 0;
      p.y = radius * sin((double)(i) / boost::math::constants::pi<double>());
      p.z = radius * cos((double)(i) / boost::math::constants::pi<double>());
    case Axis::y:
      p.x = radius * sin((double)(i) / boost::math::constants::pi<double>());
      p.y = 0;
      p.z = radius * cos((double)(i) / boost::math::constants::pi<double>());
    case Axis::z:
       p.x = radius * sin((double)(i) / boost::math::constants::pi<double>());
       p.y = radius * cos((double)(i) / boost::math::constants::pi<double>());
       p.z = 0;
    }
    controlVerticles.append(MPoint(p));
  }
  for (auto i = 0; i < nknots; i++)
    knotSequences.append((double)i);

  MObject circle = circleFn.create(controlVerticles, knotSequences,
                                   3, MFnNurbsCurve::kOpen,
                                   false, false, parent, &status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  return circle;
}

void addControllers(const PlugsMap &plugsMap, double radius) {
  for (auto it = plugsMap.begin(); it != plugsMap.end(); it++) {
    ScrewAxis sa = screwAxisFromMayaTransformAttrName(it->second.partialName());
    Axis axis = toAxis(sa);
    MObject transformObj = it->second.node();
    addNurbsCircle(axis, radius, transformObj);
  }
}
}