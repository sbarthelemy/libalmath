#ifndef SOFTBANKROBOTICSMAYAPLUGIN_SRC_MAYAURDF_H
#define SOFTBANKROBOTICSMAYAPLUGIN_SRC_MAYAURDF_H

#include <maya/MObject.h>
#include <maya/MVector.h>
#include <maya/MEulerRotation.h>
#include <almath/scenegraph/urdf.h>
#include <map>
#include <string>
#include <functional>

namespace SBRMP {

inline
MVector toMayaVector(const AL::urdf::Array3d &v) {
  return MVector(v[0], v[1], v[2]);
}

inline
MVector toMayaTranslation(const AL::urdf::Pose &pose) {
  // Maya uses centimeters, urdf uses meters
  return 100 * toMayaVector(pose.xyz());
}

inline
MEulerRotation toMayaRotation(const AL::urdf::Pose &pose) {
  // Maya and urdf both use radians
  return MEulerRotation(pose.rpy()[0], pose.rpy()[1], pose.rpy()[2],
                        MEulerRotation::kXYZ);
}

// Create a IkJoint node corresponding to the urdf joint, as a child of
// parent.
//
// The node is created so that the URDF joint freedom axes corresponds to
// node attributes ("rx", "ry", "rz", "tx", "ty", "tz")
//
// All the attributes are locked except the one corresponding to
// the URDF joint freedom axes.
//
// The joints limits are set when needed.
//
// The joint name, child_link and parent_link are not considered.
//
// Throws for some joint types and axis which are not supported yet.
MObject toMayaIkJoint(const AL::urdf::Joint &joint,
                      MObject parent = MObject::kNullObj);

// Create a Transform node corresponding to the urdf joint, as a child of
// parent.
//
// All attributes are locked.

// The joint name, child_link, parent_link, type, axis and limits are not
// considered.
MObject toMayaTransform(const AL::urdf::Joint &joint,
                        MObject parent = MObject::kNullObj);

// Index of the scene created when importing urdf joints
struct UrdfRobotJointsIndex {
  // urdf link name -> maya IkJoint
  std::map<std::string, MObject> links;
  // We could have more indexes, like
  //
  //   // urdf joint name -> maya IkJoint
  //   std::map<std::string, MObject> joints;
  //
  //   // urdf joint name -> maya IkJoint animatable plug
  //   PlugsMap dofs;
};

inline bool returnTrue(const AL::urdf::ptree &j) { return true; }

using AsIkJointPred = std::function<bool(const AL::urdf::ptree &)>;

// Import joints from the urdf and return an index
// Joints are represented in Maya as a Transform or as an IkJoint, depending
// on the return value of the asIkJoint predicate function.
//
// The IkJoint nodes  have all their attributes locked except those
// corresponding to freedom axes of the URDF joint.
//
// The Transform nodes have all their attributes locked, irrespective of
// the corresponding URDF joint type and freedom axes.
//
// Note: mimic tags are ignored
UrdfRobotJointsIndex importUrdfRobotJoints(
  const AL::urdf::RobotTree &urdfTree,
  AsIkJointPred asIkJoint,
  MObject parent = MObject::kNullObj);

// Return the "resolved filename" from the URDF filename URI.
// only URIs starting with "file:///" are supported. For instance:
// file:////usr/share/alrobotmodel/mymesh.dae or
// file:///C:/alrobotmodel/mymesh.dae
// Throw if the input is unsupported or if the file is not found.
MString findExistingFileFromUrdfUri(const std::string &filename_uri);

// Import visuals from the urdf into the indexed scene.
// Note: only meshes are supported, others geometries are skipped
void importUrdfVisuals(const AL::urdf::RobotTree &urdfTree,
                       const UrdfRobotJointsIndex &jointsIndex);
}
#endif
