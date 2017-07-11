#include "mayaurdf.h"
#include "utils.h"
#include <maya/MFStream.h>
#include <maya/MFileObject.h>
#include <boost/math/constants/constants.hpp>
#include <maya/MQuaternion.h>
#include <maya/MNamespace.h>

using AL::urdf::RobotTree;

namespace {

using namespace SBRMP;

std::pair<SBRMP::Axis, MQuaternion> toAxisAndScaleOrientation(const AL::urdf::Array3d &axis) {
  using Pair = std::pair<SBRMP::Axis, MQuaternion>;
  if (axis == AL::urdf::Array3d({ 0., 0., 1. })) {
    return Pair(SBRMP::Axis::z, MQuaternion(0., 0., 0., 1.));
  } else if (axis == AL::urdf::Array3d({ 0., 0., -1. })) {
    // rotation of pi around x, so that axis -z becomes axis z
    return Pair(SBRMP::Axis::z, MQuaternion(1., 0., 0., 0.));
  } else if (axis == AL::urdf::Array3d({ 0., 1., 0. })) {
    return Pair(SBRMP::Axis::y, MQuaternion(0., 0., 0., 1.));
  } else if (axis == AL::urdf::Array3d({ 0., -1., 0. })) {
    // rotation of pi around x, so that axis -y becomes axis y
    return Pair(SBRMP::Axis::y, MQuaternion(1., 0., 0., 0.));
  } else if (axis == AL::urdf::Array3d({ 1., 0., 0. })) {
    return Pair(SBRMP::Axis::x, MQuaternion(0., 0., 0., 1.));
  } else if (axis == AL::urdf::Array3d({ -1., 0., 0. })) {
    // rotation of pi around y, so that axis -x becomes axis x
    return Pair(SBRMP::Axis::x, MQuaternion(0., 1., 0., 0.));
  }
  throw std::invalid_argument("unaligned axis is not supportd");
}

class Visitor : public RobotTree::JointConstVisitor {
public:
  UrdfRobotJointsIndex index;
  SBRMP::AsIkJointPred asIkJoint;

  Visitor(SBRMP::AsIkJointPred asIkJoint, const std::string &rootLinkName,
          MObject rootTransformObj)
    : asIkJoint(asIkJoint) {
      index.links.emplace(rootLinkName, rootTransformObj);
    }
  bool discover(const ptree &joint_pt) {
    MStatus status;
    const auto joint = AL::urdf::Joint(joint_pt);
    const auto joint_name = joint.name();
    const auto parent_link_name = joint.parent_link();

    MObject parent = MObject::kNullObj;
    const auto it = index.links.find(parent_link_name);
    if (it != index.links.end()) {
      parent = it->second;
    }
    //index.dofs.emplace(joint_name, newObj) // TODO: pass an iterator
    // TODO try{}
    MFnTransform transformFn(
      asIkJoint(joint_pt) ? toMayaIkJoint(joint, parent)
                          : toMayaTransform(joint, parent));
    transformFn.setName(joint_name.c_str(), &status);
    CHECK_MSTATUS(status);
    transformFn.setLocked(true);
    index.links.emplace(joint.child_link(), transformFn.object());
    return true;
  }

  void finish(const ptree &joint) {
  }
};
}

namespace SBRMP {

MObject toMayaTransform(const AL::urdf::Joint &joint, MObject parent) {
  const auto origin_urdf = joint.origin();
  auto origin_xyz = toMayaTranslation(origin_urdf);
  auto origin_rpy = toMayaRotation(origin_urdf);
  MFnTransform transformFn;
  MStatus status;
  auto newObj = transformFn.create(parent, &status);
  CHECK_MSTATUS(status);
  status = transformFn.setTranslation(origin_xyz, MSpace::kTransform);
  CHECK_MSTATUS(status);
  status = transformFn.setRotation(origin_rpy);
  CHECK_MSTATUS(status);
  status = SBRMP::lockTransformPlugs(transformFn);
  CHECK_MSTATUS(status);
  return transformFn.object();
};

// TODO: cleanup this doc
//
// Urdf Joint and Link
//
// x_parent_link = H_joint_origin_transl * H_joint_origin_rot * H_joint_rotation(angle) * x_child_link
//
//
// From MFnIKJoint doc:
//
// matrix = [S] * [RO] * [R] * [JO] * [IS] * [T]
//
// [S]: scale
// [RO]: rotateOrient(attribute name is rotateAxis)
// [R]: rotate
// [JO]: jointOrient
// [IS]: parentScaleInverse
// [T]: translate
//
// From MFnTransform doc:
//
// [Sp].inv() * [S] * [Sh] * [Sp] * [St] * [Rp].inv() * [Ro] * [R] * [Rp] * [Rt] * [T]
//
//From URDF:
//  x_parent_link = T_origin_transl * R_origin_rot * R_axis(angle) * x_child_link
//    using Maya's post-multiplied matrices convention:
//    x_parent_link = x_child_link *  R_axis(angle) * R_origin_rot * T_origin_transl
//from Maya transform
//  file :///Users/seb/ar/2016/maya/MayaHelp2017_enu/index.html#!/url=./Nodes/Nodes_index.html#./Nodes/transform.html
//  http ://help.autodesk.com/view/MAYAUL/2016/ENU/?guid=__cpp_ref_class_m_transformation_matrix_html
//
//  matrix = SP.inv() * S * SH * SP * ST * RP.inv() * RA * R * RP * RT * T
//  SP, S, SH, SP, ST : scale and shear
//  R : the rotation
//  T : the translation
//  Let's rewrite it using those notations:
//  x_... : point
//  S_... : scale
//  H_... : transform
//  R_.... : rotation
// T_.... : translation
//  SH_... : shear
//  matrix = (T_SP.inv() * S_S * SH_SH * T_SP) * T_ST * (T_RP.inv() * R_RA * R_R * T_RP) * T_RT * T_T
//  | | \ - MFnTransform::getTranslation
//  | \ - MFnTransform::getRotation
//  \ - MFnTransform::rotateOrientation
//  How does it map ?
//  matrix = (R_RA * R_R)        * T_T
//  I think what I wanted was more R_RA * R_R * R_RA.inv()
//  so,
//  T_T = T_origin_transl
//  http ://help.autodesk.com/view/MAYAUL/2016/ENU/?guid=__cpp_ref_class_m_fn_ik_joint_html
//  matrix = S * RO * R * JO * IS * T
//  matrix = H_S * R_RO * R_R * R_JO * H_IS * T_T
//
//matrix = S_S * R_RO * R_R * R_JO * S_IS * T_T
//         |     |      |     |      |      \ - MFnTransform::getTranslation
//         |     |      |     |      \ - inv(MFnTransform::getScale) on the parent transform
//         |     |      |     \ - MFnIKJoint::getOrientation == jointOrient attr
//         |     |      \ - MFnTransform::getRotation
//         |     \ - MFnIKJoint::getScaleOrientation == MFnTransform::rotateOrientation == attr rotateAxis
//         \ - MFnIKJoint::getSegmentScale == MFnTransform::getScale
//  [S] : scale
//  [RO] : rotateOrient(attribute name is rotateAxis)
//  [R] : rotate
//  [JO] : jointOrient
//  [IS] : parentScaleInverse
//  [T] : translate
MObject toMayaIkJoint(const AL::urdf::Joint &joint, MObject parent) {
  const auto origin_urdf = joint.origin();
  auto origin_xyz = toMayaTranslation(origin_urdf);
  auto origin_rpy = toMayaRotation(origin_urdf);
  MFnIkJoint ikJointFn;
  MStatus status;
  auto newObj = ikJointFn.create(parent, &status);
  CHECK_MSTATUS(status);
  status = ikJointFn.setTranslation(origin_xyz, MSpace::kTransform);
  CHECK_MSTATUS(status);
  status = ikJointFn.setOrientation(origin_rpy);
  CHECK_MSTATUS(status);

  if ((joint.type() == AL::urdf::Joint::fixed) ||
    (joint.type() == AL::urdf::Joint::planar) ||
    (joint.type() == AL::urdf::Joint::floating)) {
    // TODO: avoid locking planar and floating joints
    status = SBRMP::lockIkJointPlugs(ikJointFn);
    CHECK_MSTATUS(status);
    const auto isFree = false;
    auto status = ikJointFn.setDegreesOfFreedom(isFree, isFree, isFree);
    CHECK_MSTATUS(status);
  } else {
    // these joints have a single dof, along an axis
    SBRMP::Axis axis;
    MQuaternion q(0., 0., 0., 1.);
    std::tie(axis, q) = toAxisAndScaleOrientation(joint.axis());
    // Note: setScaleOrientation also alters ikJointFn.setOrientation
    // after the setScaleOrienttaion(q) call, we'll have
    // ikJointFn.getOrientation() == q.inv() * origin_rpy.asQuaternion()
    status = ikJointFn.setScaleOrientation(q);
    CHECK_MSTATUS(status);
    auto saxis = SBRMP::rotationDof(axis);
    if (joint.type() == AL::urdf::Joint::prismatic) {
      saxis = SBRMP::translationDof(axis);
    }
    status = SBRMP::lockIkJointPlugs(ikJointFn);
    CHECK_MSTATUS(status);
    status = SBRMP::unlockTransformPlug(ikJointFn, saxis);
    CHECK_MSTATUS(status);
    status = ikJointFn.setDegreesOfFreedom(
      saxis == SBRMP::ScrewAxis::rx,
      saxis == SBRMP::ScrewAxis::ry,
      saxis == SBRMP::ScrewAxis::rz);
    CHECK_MSTATUS(status);
    if ((joint.type() == AL::urdf::Joint::revolute) ||
      (joint.type() == AL::urdf::Joint::prismatic)) {
      status = setLimits(ikJointFn, saxis, *(joint.limit_lower_upper()));
      CHECK_MSTATUS(status);
    }
  }
  return newObj;
}

UrdfRobotJointsIndex importUrdfRobotJoints(const AL::urdf::RobotTree &urdfTree,
                                           AsIkJointPred asIkJoint,
                                           MObject parent) {
  Visitor visitor(asIkJoint, urdfTree.root_link(), parent);
  urdfTree.traverse_joints(visitor);
  return std::move(visitor.index);
}

MString findExistingFileFromUrdfUri(const std::string &filename_uri) {
  auto exists = false;
  const auto uri_prefix = std::string("file:///");
  auto ps = uri_prefix.size();
  if (filename_uri.compare(0, ps, uri_prefix) != 0) {
    std::stringstream msg;
    msg << "could not locate URI \"" << filename_uri
      << "\". Unsupported URI scheme.";
    throw std::runtime_error(msg.str());
  }
  // may be relative or absolute
  auto unresolved_filename = filename_uri.substr(ps, filename_uri.size() - ps);
  // should be absolute or empty
  auto resolved_filename =
    MFileObject::getResolvedFullName(unresolved_filename.c_str(),
    exists,
    MFileObject::kInputFile);
  if (!exists) {
    std::stringstream msg;
    msg << "could not locate URI \"" << filename_uri << "\".";
    if (resolved_filename.length() > 0u) {
      msg << " tried at \"" << resolved_filename << "\"";
    }
    throw std::runtime_error(msg.str());
  }
  return resolved_filename;
}

void importUrdfVisuals(const AL::urdf::RobotTree &urdfTree,
  const UrdfRobotJointsIndex &jointsIndex) {
  // map: mesh filename -> Transform object
  // used to avoid importing several times the same mesh
  // if it appears several times in the urdf.
  std::map<std::string, MObject> meshes;
  MString filename;
  MStatus status;
  for (const auto &link : jointsIndex.links) {
    for (const auto &visual : AL::urdf::Link(urdfTree.link(link.first)).visuals()) {
      try {
        // will throw if the visual geometry is not a mesh
        const auto &meshUrdf = boost::get<AL::urdf::Mesh>(visual.geometry());
        // will throw if the filename does not match
        filename = findExistingFileFromUrdfUri(meshUrdf.filename());
        auto filename_str = std::string(filename.asChar());
        auto it = meshes.find(filename_str);
        auto duplicateMesh = false;
        MObject mesh;
        if (it != meshes.end()) {
          duplicateMesh = true;
        } else {
          // ugly, we use the fully resolved filename as the name of the transform
          const auto & name = filename;
          status = SBRMP::importColladaMesh(filename, name, mesh);
          if ((!status) || mesh.isNull()) {
            std::stringstream msg;
            msg << "Failed to import \"" << filename << "\"";
            throw std::runtime_error(msg.str());
          }
          it = meshes.emplace(filename_str, mesh).first;
          // tweak "display override" settings
          status = setAttr<bool>(it->second, "overrideEnabled", true);
          CHECK_MSTATUS_AND_THROW_IT(status);
          status = setAttr<int>(it->second, "overrideDisplayType", 2);
          CHECK_MSTATUS_AND_THROW_IT(status);
        }
        mesh = it->second;

        const auto visualOriginUrdf = visual.origin();
        const auto meshScaleUrdf = meshUrdf.scale();
        auto meshParentObj = link.second;
        if (!AL::urdf::is_identity(visualOriginUrdf) ||
            !AL::urdf::is_ones(meshScaleUrdf)) {
          MFnTransform meshOriginFn;
          meshParentObj = meshOriginFn.create(link.second);
          meshOriginFn.setTranslation(toMayaTranslation(visualOriginUrdf), MSpace::kTransform);
          meshOriginFn.setRotation(toMayaRotation(visualOriginUrdf));
          meshOriginFn.setScale(meshScaleUrdf.data());
        }
        status = MFnDagNode(meshParentObj).addChild(mesh,
                                                    MFnDagNode::kNextPos,
                                                    duplicateMesh);
        CHECK_MSTATUS(status);
      } catch (const std::exception &e) {
        std::cerr << e.what() << "\n";
      }
    }
  }
}
}
