/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

// Implementation of the RigidBodySystemBuilder interface
// (as in the "builder design pattern") which describes/export
// a rigid body system to URDF.

#ifndef LIB_ALMATH_SCENEGRAPH_URDFRIGIDBODYSYSTEMBUILDER_H
#define LIB_ALMATH_SCENEGRAPH_URDFRIGIDBODYSYSTEMBUILDER_H

#include <almath/scenegraph/rigidbodysystembuilder.h>
#include <almath/scenegraph/urdf.h>
#include <almath/scenegraph/urdfeigen.h>

namespace AL {
namespace Math {

// RigidBodySystemBuilder::Interface<float> &builder

template <typename Scalar>
class UrdfRigidBodySystemBuilder :
    public RigidBodySystemBuilder::Interface<Scalar> {
  const RigidBodySystemBuilder::Config &config() const {
    return _conf;;
  }
private:
  RigidBodySystemBuilder::Config _conf;
  urdf::ptree _robot;
public:
  using typename RigidBodySystemBuilder::Interface<Scalar>::Link;
  using typename RigidBodySystemBuilder::Interface<Scalar>::StaticFrame;

  UrdfRigidBodySystemBuilder() {

  }
  urdf::ptree robot() {
    return _robot;
  }

  void addLink(Link link) {
    bool implicit_joint = false;
    if (link.parent_body == _conf.galilean_frame) {
      if (link.joint_type != RigidBodySystemBuilder::JointType::FreeFlyer) {
        throw std::runtime_error(
            "only freeflyer joints can have a galilean parent");
      }
      implicit_joint = true;
    }
    auto &ulink = urdf::robot::add_link(_robot, link.new_body);
    if (link.body_mass.mass != 0) {
      auto &inertial = urdf::link::put_inertial(ulink,
                               link.body_mass.mass,
                               link.body_mass.rotational_inertia(0, 0),
                               link.body_mass.rotational_inertia(0, 1),
                               link.body_mass.rotational_inertia(0, 2),
                               link.body_mass.rotational_inertia(1, 1),
                               link.body_mass.rotational_inertia(1, 2),
                               link.body_mass.rotational_inertia(2, 2));
      const auto origin = Math::urdfPoseFromEigenTransform(
            Eigen::Isometry3d(Eigen::Translation<Scalar, 3>(
                                link.body_mass.center_of_mass))).to_ptree();
      if (origin)
        inertial.put_child("origin", *origin);
    }
    if (implicit_joint)
      return;
    auto &ujoint = urdf::robot::add_joint(_robot, link.new_joint);
    ujoint.put("parent.<xmlattr>.link", link.parent_body);
    ujoint.put("child.<xmlattr>.link", link.new_body);
    auto opt_origin =
        Math::urdfPoseFromEigenTransform(link.pose_parent_new).to_ptree();
    if (opt_origin)
      ujoint.put_child("origin", *opt_origin);
    {
      // deal with joint type and axis
      std::string type = "continuous";
      boost::optional<std::string> axis;
      switch (link.joint_type) {
      case RigidBodySystemBuilder::JointType::Rx:
        axis = "1 0 0";
        break;
      case RigidBodySystemBuilder::JointType::Ry:
        axis = "0 1 0";
        break;
      case RigidBodySystemBuilder::JointType::Rz:
        axis = "0 0 1";
        break;
      case RigidBodySystemBuilder::JointType::FreeFlyer:
        type = "floating";
        break;
      }
      ujoint.put("<xmlattr>.type", type);
      if (axis)
        ujoint.put("axis.<xmlattr>.xyz", *axis);
    }
  }

  void addStaticFrame(StaticFrame sframe) {
    if (sframe.parent_frame == _conf.galilean_frame) {
      throw std::runtime_error(
          "only freeflyer joints can have a galilean parent");
    }
    urdf::robot::add_link(_robot, sframe.new_static_frame);
    auto &ujoint = urdf::robot::add_joint(_robot, sframe.new_static_transform);
    ujoint.put("parent.<xmlattr>.link", sframe.parent_frame);
    ujoint.put("child.<xmlattr>.link", sframe.new_static_frame);
    auto opt_origin =
        Math::urdfPoseFromEigenTransform(sframe.pose_parent_new).to_ptree();
    if (opt_origin)
      ujoint.put_child("origin", *opt_origin);
    ujoint.put("<xmlattr>.type", "fixed");
  }
};
}
}
#endif
