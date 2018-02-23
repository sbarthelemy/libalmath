/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_URDF_H
#define LIB_ALMATH_SCENEGRAPH_URDF_H

#include <almath/api.h>
#include <string>
#include <iosfwd>
#include <memory>
#include <array>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/function/function_fwd.hpp>
#include <boost/ref.hpp>
#include <boost/tokenizer.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <set>
#include <vector>
#include <type_traits>
#include <boost/property_tree/stream_translator.hpp>

namespace boost {
namespace property_tree {

// hack around ptree bug #10188 [1]: for floating point numbers they don't
// serialize with enough precision:
// they use digits10+1 instead of max_digits10
// [1] https://svn.boost.org/trac/boost/ticket/10188
template <typename Ch, typename Traits>
struct customize_stream<Ch, Traits, double, void> {
  static void insert(std::basic_ostream<Ch, Traits> &s, const double &e) {
    s.precision(std::numeric_limits<double>::max_digits10);
    s << e;
  }
  static void extract(std::basic_istream<Ch, Traits> &s, double &e) {
    s >> e;
    if (!s.eof()) {
      s >> std::ws;
    }
  }
};
}
}

namespace AL {

namespace urdf {

typedef boost::property_tree::ptree ptree;

namespace detail {
class RobotTreeP;
}

typedef std::array<double, 3> Array3d;
typedef std::array<double, 4> Array4d;

inline bool is_zero(const Array3d &a) {
  return (a[0] == 0) && (a[1] == 0) && (a[2] == 0);
}

inline bool is_ones(const Array3d &a) {
  return (a[0] == 1) && (a[1] == 1) && (a[2] == 1);
}

class Pose;

// return the name attribute of a joint or link element
ALMATH_API std::string name(const ptree &pt);
inline std::string get_name(const ptree &pt) {
  return name(pt);
}

// return the parent link of a joint element
ALMATH_API std::string parent_link(const ptree &pt);

// return the child link of a joint element
ALMATH_API std::string child_link(const ptree &pt);

// A parser for urdf xml files [1].
//
// Contrary to the urdf parser from ROS [2], [3], this parser:
//
// * preserves joint ordering defined in XML file. This is useful if one
//   wants to use this ordering to define the joint indexes in a
//   state vector.
//
// * provides access to the raw XML elements, so that you can leverage the
//   parser to handle unofficial URDF extensions and
//   to modify the XML tree and write it back while altering the document as
//   little as possible.
//
// * does not systematically convert roll-pitch-yaw angles to quaternions,
//   which saves us some bugs and better fits the "alter the document as
//   little as possible" philosophy.
//
//  [1] http://wiki.ros.org/urdf/XML
//  [2] https://github.com/ros/robot_model
//  [3] https://github.com/ros/urdfdom_headers
//
// The UrdfTree class acts as an index for the boost::property_tree of the
// (URDF) XML document.
// Given the ptree of the robot XML element, it traverses the tree and
// indexes the ptree elements for the URDF joints and links.
//
// The class is akin to a set of iterators: it does not copy nor own the
// XML tree, and may be invalidated if the XML tree changes.
//
// The user shall ensure that the XML root and the joint and link ptree
// elements stay alive and valid for the lifetime of this RobotTree.
// For instance the user may use direct access to the ptree elements to alter
// the mass of a link, but shall no delete the link, otherwise calls to
// RobotTree::link("my_link_name") would use dandling pointers and return
// invalid references.
//
// On the other hand, some RobotTree functions enable the safe modification or
// removal of joints and links.
class ALMATH_API RobotTree {
 public:
  // Read a XML tree given the robot element.
  // The ptree is not copied, a reference is kept instead.
  // The user shall ensure that the reference is valid for the whole life
  // of the RobotTree object.
  RobotTree(ptree &robot);
  ~RobotTree();
  RobotTree(RobotTree const &) = delete;
  void operator=(RobotTree const &other) = delete;

  const ptree &link(const std::string &name) const;
  ptree &link(const std::string &name);

  const ptree &joint(const std::string &name) const;
  ptree &joint(const std::string &name);

  const std::string &root_link() const;

  // If there is a single root joint, remove it and its parent link, throw otherwise.
  //
  // References and iterators to the removed link and joint are invalidated.
  void rm_root_joint();

  // If the given joint is a leaf in the kinematic tree,
  // remove it and its child link, throw otherwise.
  //
  // References and iterators to the removed link and joint are invalidated.
  void rm_leaf_joint(const std::string &name);

  // Change the root link frame of reference.
  //
  // Each urdf link has an implicitly defined frame of reference with respect
  // to which the pose of the link children elements (inertia, collisions,
  // visuals) and joints are defined.
  //
  // This function changes the root link frame of reference, while updating
  // all its children elements and joints accordingly so that the physical
  // system being described does not change.
  //
  // The function takes one argument "pose" which is used to define the
  // transform from the old frame of reference of the new one according to
  // the formula:
  //
  //   point_in_new_frame = pose * point_in_old_frame
  //
  // children elements and joints poses are updated using:
  //
  //   child_pose_in_new_frame = pose.inverse() * child_pose_in_old_frame
  //
  // Note: changing the frame of reference of a link which is not the root one
  // would also be possible, but only to some extent since its origin must
  // lie on the parent joint axis.
  void transport_root_link_frame(const Pose &pose);

  // Modify the kinematic tree so that the given link is the root without
  // changing the physical meaning of the described system.
  //
  // Flips all the joints on the path between the old and the new root.
  //
  // Throws is there is a multi-dof joint on the path, because flipping it
  // would change the joint-space parametrization.

  // Change the frame of reference of the links involved so that its
  // origin lies on the parent joint axis.
  //
  // Let consider a root link "a" with children links "b" and "c" and
  // joints "ab" and "bc".
  //
  // The kinematic tree can be written as:  a --ab--> b
  //                                          +-ac--> c
  //
  // The frame may look like:
  //                                                / \    .
  //                                               /   \   .
  //                                      / \     /    /   .
  //                                     /   \   /    /    .
  //                                  --/ \/ /--/ \/ /--   .
  //                         __________/ ab /   \ b /      .
  //                        /              /     \ /       .
  //                       /     a |_     /                .
  //              / \     /    __________/                 .
  //             /   \   /    /                            .
  //          --/ \/ /--/ \/ /--                           .
  //           /  c /   \ ac/                              .
  //          /    /     \ /                               .
  //          \   /                                        .
  //           \ /                                         .
  //
  // After calling define_as_root_link("b")
  // * the kinematic tree can be written as
  //   b --ab--> a --ac--> c
  // * the reference frame of "b" and "c" are unchanged
  // * the reference frame of "a" moved to joint "ab" frame, so as to lie
  //   on joint "ab" axis.
  // * joint "ab" origin pose is now defined with respecte to link "b"
  //   frame and is thus the identity
  // * joint "ac" origin pose has been updated to acount fot the
  //
  // The frames look like:
  //                                                / \       .
  //                                               /   \      .
  //                                      / \     /    /      .
  //                                     /   \   /    /       .
  //                                  --/ \/ /--/ \/ /--      .
  //                         __________/  a /   \ b / == ba   .
  //                        /              /     \ /          .
  //                       /              /                   .
  //              / \     /    __________/                    .
  //             /   \   /    /                               .
  //          --/ \/ /--/ \/ /--                              .
  //           /  c /   \ ac/                                 .
  //          /    /     \ /                                  .
  //          \   /                                           .
  //           \ /                                            .
  void define_as_root_link(const std::string &name);

  class JointConstVisitor {
   public:
    // a false return value stops the traversal for the current branch
    virtual bool discover(const ptree &) { return true; }
    virtual void finish(const ptree &) {}
  };
  class JointVisitor {
   public:
    virtual bool discover(ptree &) { return true; }
    virtual void finish(ptree &) {}
  };

  // Traverse the kinematic tree with a depth first traversal.
  // Siblings joints are visited using the ordering from the urdf file.
  void traverse_joints(JointConstVisitor &visitor) const;
  void traverse_joints(JointVisitor &visitor);

  bool is_mimic_tree_flat() const;

 private:
  std::unique_ptr<detail::RobotTreeP> _p;
};

typedef RobotTree::JointConstVisitor JointConstVisitor;
typedef RobotTree::JointVisitor JointVisitor;

// Convenience wrapper classes around URDF ptree elements

// helper to convert ptree to/from an array of floating point numbers
template <typename Scalar, std::size_t N>
struct ScalarArrayTranslator {
  typedef std::string internal_type;
  typedef std::array<Scalar, N> external_type;
  static_assert(std::is_scalar<Scalar>::value,
                "ScalarArrayTranslator requires a scalar");
  boost::optional<external_type> get_value(const internal_type &str) {
    // str is expected to hold N floating point numbers.
    // let split it into N strings, then use the usual boost::property_tree
    // translator to convert each substring into a Scalar.
    // In case of failure, return an uninitialized boost::optional, like
    // boost::property_tree does itself.
    boost::tokenizer<> tok(
        str, boost::char_delimiters_separator<char>(false, "", " \t\n\v\f\r"));
    boost::optional<Scalar> d;
    // note: maybe the tr variable could be made static const?
    typename
    boost::property_tree::translator_between<internal_type, Scalar>::type tr;
    external_type x;
    boost::tokenizer<>::iterator beg = tok.begin();
    size_t i = 0;
    for (; beg != tok.end() && i < N; ++beg, ++i) {
      d = tr.get_value(*beg);
      if (!d) return boost::optional<external_type>();
      x[i] = *d;
    }
    if (i != N || beg != tok.end())
      return boost::optional<external_type>();
    return boost::optional<external_type>(x);
  }

  boost::optional<internal_type> put_value(const external_type &v) {
    // note: maybe the tr variable could be made static const?
    typename
    boost::property_tree::translator_between<internal_type, Scalar>::type tr;
    std::ostringstream ss;
    if (N > 0u)
      ss << *tr.put_value(v[0]);
    for (auto i=1u; i < N; ++i) {
      ss << " " << *tr.put_value(v[i]);
    }
    return ss.str();
  }
};

using Array3dTranslator = ScalarArrayTranslator<double, 3>;
using Array4dTranslator = ScalarArrayTranslator<double, 4>;

// Models an URDF pose/transform ("origin" XML element)
class ALMATH_API Pose {
 public:
  Pose(const Array3d &xyz, const Array3d &rpy) : _xyz(xyz), _rpy(rpy) {}
  Pose() : Pose({{0, 0, 0}}, {{0, 0, 0}}) {}
  const Array3d &xyz() const { return _xyz; }
  const Array3d &rpy() const { return _rpy; }
  Pose inverse() const;
  ALMATH_API friend Pose operator*(const Pose &lhs, const Pose &rhs);

  static Pose from_ptree(const ptree &pt);
  static Pose from_ptree(const boost::optional<const ptree &> &pt);
  boost::optional<ptree> to_ptree() const;

 private:
  Array3d _xyz;
  Array3d _rpy;
};

inline bool operator==(const Pose &lhs, const Pose &rhs) {
  return (lhs.xyz() == rhs.xyz()) && (lhs.rpy() == rhs.rpy());
}
inline bool operator!=(const Pose &lhs, const Pose &rhs) {
  return !(lhs == rhs);
}

inline bool is_identity(const Pose &p) {
  return is_zero(p.xyz()) && is_zero(p.rpy());
}

// Convenience wrapper around an URDF mimic XML element
class ALMATH_API Mimic {
 public:
  const ptree &pt;
  Mimic(const ptree &pt);
  std::string joint() const;
  double multiplier() const;
  double offset() const;
};

// Convenience wrapper around an URDF joint XML element
class ALMATH_API Joint {
 public:
  enum Type { revolute, continuous, prismatic, fixed, floating, planar };
  const ptree &pt;
  Joint(const ptree &pt);
  std::string name() const;
  Type type() const;
  std::string parent_link() const;
  std::string child_link() const;
  Pose origin() const;
  Array3d axis() const;

  // Return the (lower, upper) limits pair.
  //
  // If both limits are missing, the returned value is uninitialized
  // If one of them is missing, it is defaulted to 0.
  // This behavior is slightly different from the
  // http://wiki.ros.org/urdf/XML/joint spec.
  // Beware, there is no warranty that lower <= upper.
  boost::optional<std::pair<double, double>> limit_lower_upper() const;
  boost::optional<double> limit_effort() const;
  boost::optional<double> limit_velocity() const;
  boost::optional<Mimic> mimic() const;
};

// Convenience wrapper around an URDF inertial XML element
class ALMATH_API Inertial {
 public:
  const ptree &pt;
  Inertial(const ptree &pt);
  Pose origin() const;
  double mass() const;
  double ixx() const;
  double ixy() const;
  double ixz() const;
  double iyy() const;
  double iyz() const;
  double izz() const;
};

class ALMATH_API Box {
 public:
  const ptree &pt;
  Box(const ptree &pt);
  Array3d size();
};

// z-axis cylinder, centered at the origin
class ALMATH_API Cylinder {
 public:
  const ptree &pt;
  Cylinder(const ptree &pt);
  double radius();
  double length();
};

class ALMATH_API Sphere {
 public:
  const ptree &pt;
  Sphere(const ptree &pt);
  double radius();
};

class ALMATH_API Mesh {
 public:
  const ptree &pt;
  Mesh(const ptree &pt);
  std::string filename() const;
  // Note: scale element is optional. When absent, we return (1, 1, 1)
  Array3d scale() const;
};

using Geometry = boost::variant<Box, Cylinder, Sphere, Mesh>;

class ALMATH_API Visual {
public:
  const ptree &pt;
  Visual(const ptree &pt);
  Pose origin() const;
  Geometry geometry() const;

  static bool is_visual(const ptree::value_type &val);
};

// Convenience wrapper around an URDF link XML element
class ALMATH_API Link {
 public:
  const ptree &pt;
  Link(const ptree &pt);
  std::string name() const;
  boost::optional<Inertial> inertial() const;

 private:
  // helper: return the range of visual ptree children
  inline auto _visual_ptrees() const
  -> boost::select_second_const_range<
        decltype(boost::adaptors::filter(pt, Visual::is_visual))> {
  return boost::adaptors::values(
        boost::adaptors::filter(pt, Visual::is_visual));
  }
  // helper, call the Visual ctor
  static inline Visual _makeVisual(const ptree &pt) { return Visual(pt); }

 public:
  // return the range of Visual children
  inline auto visuals() const
  -> boost::transformed_range<
        decltype(&_makeVisual),
        const decltype(_visual_ptrees())> {
    return boost::adaptors::transform(_visual_ptrees(), &_makeVisual);
  }
};

// Utils

// make the type of the joint named "name" equal to "fixed",
// and erase the joint axis and limits, if any.
ALMATH_API void makeJointFixed(RobotTree &parser, const std::string &name);

// make the type of the joint named "name" equal to "floating",
// and erase the joint axis and limits, if any.
ALMATH_API void makeJointFloating(RobotTree &parser, const std::string &name);

// apply makeJointFixed to all joints of type "continuous".
// Return the names of the joints whose type was changed.
ALMATH_API
std::vector<std::string> makeContinuousJointsFixed(RobotTree &parser);

// Squash a joint's child link mass into its parent link mass.
// The child link mass element (if any) is erased.
ALMATH_API void squashJointMass(RobotTree &parser, const std::string &name);

// apply squashJointMass to all fixed joints.
ALMATH_API void squashFixedJointsMass(RobotTree &parser);

// Squash all fixed joint's child link mass into their parent link mass and
// then convert all massless joints into fixed joints.
// The point is to avoid massless mobile joints, which have no physical
// meaning.
// Return the names of the joints whose type was changed.
ALMATH_API std::vector<std::string> makeMasslessJointsFixed(RobotTree &parser);

// Remove kinematic tree subtrees starting at joints for which predicate
// returns true.
//
// Return the names of the removed joints.
//
// For instance, to cut the tree at joints whose name ends with "_useless",
// one could do:
//
//   ptree robot = ...
//   auto pred = [](const ptree &joint) {
//     return boost::regex_match(urdf::name(joint), boost::regex(".*_useless$"));
//   };
//   urdf::removeSubTreeIfJoint(urdf::RobotTree(robot), pred);
ALMATH_API std::vector<std::string> removeSubTreeIfJoint(
    RobotTree &parser,
    std::function<bool(const ptree &joint)> pred);

// a visitor which prints the URDF kinematic tree as a dot graph
// when visiting its joints.
class ALMATH_API UrdfDotPrinterVisitor : public RobotTree::JointConstVisitor {
  const char tab;
  int depth;
  bool do_indent;
  std::ostream &os;

 public:
  UrdfDotPrinterVisitor(std::ostream &os);

 public:
  bool discover(const ptree &joint);
  void finish(const ptree &joint);
};


ALMATH_API void put_name(ptree &pt, const std::string &name);

namespace material {

// TODO: doc
template <typename Scalar>
typename std::enable_if<std::is_floating_point<Scalar>::value>::type
put_color(ptree &pt, const std::array<Scalar, 4> &rgba) {
  // TODO check rgba values are in [0, 1]
  pt.put("color.<xmlattr>.rgba", rgba, ScalarArrayTranslator<Scalar, 4>());
}

// TODO: get_color
// TODO: put/get _texture

}

namespace link {

ALMATH_API bool is_link(const ptree::value_type &val);

template <typename Scalar>
ptree make_inertial(Scalar mass,
                    Scalar ixx,
                    Scalar ixy,
                    Scalar ixz,
                    Scalar iyy,
                    Scalar iyz,
                    Scalar izz) {
  ptree inertial;
  inertial.put("mass.<xmlattr>.value", mass);
  inertial.put("inertia.<xmlattr>.ixx", ixx);
  inertial.put("inertia.<xmlattr>.ixy", ixy);
  inertial.put("inertia.<xmlattr>.ixz", ixz);
  inertial.put("inertia.<xmlattr>.iyy", iyy);
  inertial.put("inertia.<xmlattr>.iyz", iyz);
  inertial.put("inertia.<xmlattr>.izz", izz);
  return inertial;
}

template <typename Scalar>
ptree &put_inertial(ptree &pt,
                    Scalar mass,
                    Scalar ixx,
                    Scalar ixy,
                    Scalar ixz,
                    Scalar iyy,
                    Scalar iyz,
                    Scalar izz) {
  auto inertial = make_inertial(mass, ixx, ixy, ixz, iyy, iyz, izz);
  return pt.put_child("inertial", inertial);
}

}

namespace robot {

// return the range of link elements, in document order.
inline auto get_links(const ptree &pt)
-> boost::select_second_const_range<
      decltype(boost::adaptors::filter(pt, link::is_link))> {
  return boost::adaptors::values(
             boost::adaptors::filter(pt, link::is_link));
}
inline auto get_links(ptree &pt)
-> boost::select_second_mutable_range<
      decltype(boost::adaptors::filter(pt, link::is_link))> {
  auto kv = boost::adaptors::filter(pt, link::is_link);
  return boost::adaptors::values(kv);
}

ALMATH_API ptree &add_link(ptree &pt, const std::string &name);
ALMATH_API ptree &require_link(ptree &pt, const std::string &name);

ALMATH_API ptree &add_joint(ptree &pt, const std::string &name);

// Replace each mesh and texture filename by the result of applying op() to it.
//
// For instance, to replace .mesh extensions by .dae, one could do:
//
//   ptree robot = ...
//   auto op = [](std::string in) {
//     return boost::regex_replace(in, boost::regex("\\.mesh$"), ".dae");};
//   urdf::robot::transform_filenames(robot, op);
ALMATH_API void transform_filenames(
    ptree &robot, std::function<std::string(std::string)> op);
}

}
}

#endif
