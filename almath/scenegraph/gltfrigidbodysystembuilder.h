/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

// Implementation of the RigidBodySystemBuilder interface
// (as in the "builder design pattern") which describes/export
// a rigid body system to glTF.

#ifndef LIB_ALMATH_SCENEGRAPH_GLTFRIGIDBODYSYSTEMBUILDER_H
#define LIB_ALMATH_SCENEGRAPH_GLTFRIGIDBODYSYSTEMBUILDER_H

#include <string>
#include <map>
#include <boost/optional.hpp>
#include <almath/scenegraph/rigidbodysystembuilder.h>
#include <almath/scenegraph/gltf.h>
#include <almath/scenegraph/json.hpp>

namespace AL {
namespace Math {

// RigidBodySystemBuilder::Interface<float> &builder

class GltfRigidBodySystemBuilder :
    public RigidBodySystemBuilder::Interface<float> {
  const RigidBodySystemBuilder::Config &config() const {
    return _conf;;
  }
public:
  typedef float Scaling;
  using typename RigidBodySystemBuilder::Interface<Scalar>::Link;
  using typename RigidBodySystemBuilder::Interface<Scalar>::StaticFrame;
  using typename RigidBodySystemBuilder::Interface<Scalar>::AffineCompact3;

private:
  RigidBodySystemBuilder::Config _conf;
  glTF::Scene _scene;
  std::vector<glTF::Node> _nodes;
  std::vector<nlohmann::json::object_t> _nodez;
  std::map<std::string, size_t> _body_node_idxs;
  std::map<std::string, size_t> _joint_node_idxs;
  static bool has_name(const glTF::Node &node, const std::string &name)
  {
    const auto &opt_name = node.name;
    return opt_name && (*opt_name == name);
  }
  glTF::rst toRST(AffineCompact3 pose) {
    Eigen::Affine3f affine;
    affine = pose;
    Eigen::Matrix3f rotation_matrix;
    //Eigen::Vector3f scaling;
    Eigen::Matrix3f scaling_matrix;
    pose.computeRotationScaling(&rotation_matrix, &scaling_matrix);
    Eigen::Quaternionf rotation_quaternion;
    rotation_quaternion = rotation_matrix;
    Eigen::Vector3f scaling_vector;
    scaling_vector = scaling_matrix.diagonal();
    Eigen::Vector3f translation_vector;
    translation_vector = pose.translation();
    return glTF::rst{rotation_quaternion,
                     scaling_vector,
                     translation_vector};
  }
public:
  GltfRigidBodySystemBuilder() {

  }

  glTF::Document document() {
    glTF::Document ret;
    for (const auto &node: _nodes)
      ret.nodes.push_back(boost::lexical_cast<std::string>(node));
    return ret;
  }

  nlohmann::json json() {
    return nlohmann::json{{"nodes", _nodez}};
  }

  void addLink(Link link) {
    const auto node_idx = _nodes.size();
    bool implicit_joint = false;
    if (link.parent_body == _conf.galilean_frame) {
      if (link.joint_type != RigidBodySystemBuilder::JointType::FreeFlyer) {
        throw std::runtime_error(
            "only freeflyer joints can have a galilean parent");
      }
      implicit_joint = true;
      // TODO add to scne
    } else {
      auto parent_idx = _body_node_idxs.at(link.parent_body);
      _nodes.at(parent_idx).children.push_back(node_idx);
      _nodez.at(parent_idx)["children"].push_back(node_idx);
    }

    _body_node_idxs[link.new_body] = node_idx;
    _joint_node_idxs[link.new_joint] = node_idx;
    const auto rst = toRST(link.pose_parent_new);
    const auto node_name = implicit_joint ? link.new_body : link.new_joint;
    _nodes.push_back(glTF::Node{{},
                                node_name,
                                boost::none,
                                boost::none,
                                rst});
    const auto rotation = *(rst.rotation);
    const auto translation = *(rst.translation);
    _nodez.push_back(nlohmann::json{
                       {"name", node_name},
                       {"rotation", {rotation.x(), rotation.y(), rotation.z(), rotation.w()}},
                       //{"scale", {scale[0], scale[1], scale[2]}},
                       {"translation", {translation[0], translation[1], translation[2]}},
                       {"children", nlohmann::json::array_t{}}});
    if (implicit_joint) // TODO
      return; 
  }

  void addStaticFrame(StaticFrame sframe) {
    if (sframe.parent_frame == _conf.galilean_frame) {
      throw std::runtime_error(
          "only freeflyer joints can have a galilean parent");
    }
  }
};
}
}
#endif
