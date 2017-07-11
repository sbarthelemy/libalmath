#include <gtest/gtest.h>
#include <boost/property_tree/ptree.hpp>
#include <maya/MTime.h>
#include <maya/MLibrary.h>
#include <maya/MAngle.h>
#include <maya/MGlobal.h>
#include <maya/MNamespace.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MMatrix.h>
#include <maya/MDGModifier.h>
#include "../src/utils.h"
#include "../src/mayaurdf.h"
#include "../src/mayaqianim.h"
#include <sstream>

#include <almath/scenegraph/urdf.h>
#include <almath/scenegraph/qianim.h>

#ifdef WITH_ALMATHINTERNAL
#include <almathinternal/interpolations/bezierkeys.h>
#include <almathinternal/interpolations/alinterpolationtypes.h>
#endif

using namespace AL;

typedef AL::urdf::ptree ptree;

MTime::Unit defaultUIUnit;

std::string almodelutils_prefix_path;
std::string collada_mesh_path;

class MayaTest : public ::testing::Test {
public:
  MayaTest() {
    // Create a new empty Maya scene
    MGlobal::executeCommand("file -f -new");
    // restore default UI Unit in case previous test changed it.
    MTime::setUIUnit(defaultUIUnit);
  }
};

// Used for tests that only involve Maya code, in order to ensure
// the behaviour is the one we expect.
class MayaBasicsTest : public MayaTest {};

TEST_F(MayaBasicsTest, MTime) {
  // Maya uses MTime to represent time.
  // A MTime exposes a unit and a value.
  // The number of frame per second is given by the MTime unit
  // The value represents the time as a (double precision) floating point number of frames.
  //
  // However, internally, Maya represenets time as an integer number of frames at 6000 fps.
  // So the internal Maya computations use a fixed-point numbers.
  // See
  // "How Maya counts time"
    // http://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=GUID-A9F91D81-3149-47AF-9E37-83A89242D870

  // frame 1 at 24 fps
  EXPECT_DOUBLE_EQ(1., (MTime(1., MTime::Unit::kFilm).value()));
  // ... is equivalent to frame 1/24 at 1 fps
  EXPECT_DOUBLE_EQ(1. / 24, (MTime(1., MTime::Unit::kFilm).asUnits(MTime::Unit::kSeconds)));
  // ... is equivalent to frame 25/24 at 25 fps
  EXPECT_DOUBLE_EQ(25. / 24, (MTime(1., MTime::Unit::kFilm).asUnits(MTime::Unit::kPALFrame)));

  // frame 1 / 6000 at 1 fps is equivalent to frame 1 at 6000 fps
  EXPECT_DOUBLE_EQ(1., (MTime(1. / 6000, MTime::Unit::kSeconds).asUnits(MTime::Unit::k6000FPS)));
  // frame 1.1 / 6000 at 1 fps is *rounded* to frame 1 at 6000 fps
  EXPECT_DOUBLE_EQ(1., (MTime(1.1 / 6000, MTime::Unit::kSeconds).asUnits(MTime::Unit::k6000FPS)));
}


TEST_F(MayaBasicsTest, MTimeFromAnimCurve) {
  MFnAnimCurve acFn;
  acFn.create(MFnAnimCurve::AnimCurveType::kAnimCurveTA);

  // time of key 1
  MTime tk1(1., MTime::Unit::kPALFrame); // 25 fps
  // time of key 1 out tangent
  MTime tto1(2., MTime::Unit::kPALFrame);

  // time of key 2 in tangent
  MTime tti2(2., MTime::Unit::kFilm); // 24 fps
  // time of key 2
  MTime tk2(3., MTime::Unit::kFilm);

  auto i1 = acFn.addKey(tk1, 14.0);
  auto i2 = acFn.addKey(tk2, 42.0);
  acFn.setTangentsLocked(i1, false);
  acFn.setTangentsLocked(i2, false);

  const auto xto1 =
    static_cast<float>(3 * (tto1 - tk1).as(MTime::Unit::kSeconds));

  {
    const auto y = 0.f; // horizontal tangent
    const auto in = false;
    const auto convertUnits = false;
    const auto status = acFn.setTangent(i1, xto1, y, in, NULL, convertUnits);
    CHECK_MSTATUS(status);
  }
  const auto xti2 =
    static_cast<float>(3 * (tk2 - tti2).as(MTime::Unit::kSeconds));
  {
    const auto y = 0.f; // horizontal tangent
    const auto in = true;
    const auto convertUnits = false;
    const auto status = acFn.setTangent(i2, xti2, y, in, NULL, convertUnits);
    CHECK_MSTATUS(status);
  }
  // when MFnAnimCurve returns the key times, it has changed their
  // unit to the global ui pref.
  // However the tangents abscissa are always returned as a floating point
  // number of seconds (multiplied by 3)
  {
    const auto uiunit = MTime::uiUnit();
    EXPECT_EQ(uiunit, MTime::uiUnit());
    MTime r1 = acFn.time(i1);
    EXPECT_EQ(tk1, r1);
    EXPECT_EQ(MTime::uiUnit(), r1.unit());
    auto in = false;
    float x, y;
    acFn.getTangent(i1, x, y, in);
    EXPECT_FLOAT_EQ(xto1, x);
    EXPECT_FLOAT_EQ(0.f, y);
  }
  {
    MTime::setUIUnit(MTime::kSeconds);
    EXPECT_EQ(MTime::kSeconds, MTime::uiUnit());
    MTime r1 = acFn.time(i1);
    EXPECT_EQ(tk1, r1);
    EXPECT_EQ(MTime::uiUnit(), r1.unit());
    auto in = false;
    float x, y;
    acFn.getTangent(i1, x, y, in);
    EXPECT_FLOAT_EQ(xto1, x);
    EXPECT_FLOAT_EQ(0.f, y);
  }
}

void print(const MFnIkJoint &ikJointFn) {
  auto &os = std::cout;
  os << "getTranslation(MSpace::kTransform): " << ikJointFn.getTranslation(MSpace::kTransform) << "\n";
  MQuaternion quaternion;
  ikJointFn.getOrientation(quaternion);
  os << "getOrientation(): " << quaternion << "\n";

  ikJointFn.getRotation(quaternion);
  os << "getRotation(): " << quaternion << "\n";
  ikJointFn.getScaleOrientation(quaternion);
  os << "getScaleOrientation(): " << quaternion << "\n"
      << "rotateOrientation(MSpace::kTransform): " << ikJointFn.rotateOrientation(MSpace::kTransform) << "\n"
      << "transformation(): " << ikJointFn.transformation().asMatrix() << "\n"
      << std::endl;
}

TEST_F(MayaBasicsTest, MFnIkJoint) {
  // data copied from Juliette KneePitch joints, after setting Leg_effector as root
  auto origin_xyz = MVector(-0.0061999999999999998, 1.2268505046151292e-006, 0.33399999999774682);
  auto origin_rpy = MEulerRotation(-1.5708000000000002, -0., 0, MEulerRotation::kXYZ);
  auto axis = MVector(-0., -0., -1.);
  auto angle = MAngle(30, MAngle::kDegrees);
  {
    std::cout << "NO FLIP" << std::endl;
    MFnIkJoint ikJointFn;
    MObject ikJointObj = ikJointFn.create();
    print(ikJointFn);

    ikJointFn.setTranslation(origin_xyz, MSpace::kTransform);
    print(ikJointFn);

    ikJointFn.setOrientation(origin_rpy.asQuaternion());
    print(ikJointFn);

    ikJointFn.setRotation(
      MQuaternion(angle.as(MAngle::kRadians), axis));
    print(ikJointFn);
  }
  {
    std::cout << "FLIP" << std::endl;
    MFnIkJoint ikJointFn;
    MObject ikJointObj = ikJointFn.create();
    print(ikJointFn);

    ikJointFn.setTranslation(origin_xyz, MSpace::kTransform);
    print(ikJointFn);

    MQuaternion q(1., 0., 0., 0.);

    ikJointFn.setScaleOrientation(q);
    print(ikJointFn);

    ikJointFn.setOrientation(origin_rpy.asQuaternion() * q);
    //ikJointFn.setOrientation(q);
    print(ikJointFn);

    ikJointFn.setRotation(
      MQuaternion(angle.as(MAngle::kRadians), -axis));
    print(ikJointFn);
  }
}

class MayaUtilsTest : public MayaTest {};

TEST_F(MayaUtilsTest, check_mstatus_and_throw_it) {
  EXPECT_ANY_THROW(CHECK_MSTATUS_AND_THROW_IT(MStatus::kFailure));
  EXPECT_NO_THROW(CHECK_MSTATUS_AND_THROW_IT(MStatus::kSuccess));
}

TEST_F(MayaUtilsTest, unitFromFps) {
  EXPECT_ANY_THROW(SBRMP::unitFromFps(0));
  EXPECT_ANY_THROW(SBRMP::unitFromFps(-1));
  EXPECT_EQ(MTime::Unit::kFilm, SBRMP::unitFromFps(24));
}

TEST_F(MayaUtilsTest, fpsFromUnit) {
  EXPECT_EQ(24, SBRMP::fpsFromUnit(MTime::Unit::kFilm));

  // invalid inputs
  EXPECT_ANY_THROW(SBRMP::fpsFromUnit(MTime::Unit::kInvalid));
  EXPECT_ANY_THROW(SBRMP::fpsFromUnit(MTime::Unit::kUserDef));
  EXPECT_ANY_THROW(SBRMP::fpsFromUnit(MTime::Unit::kLast));

  // invalid inputs: cannot return 1/60 as an int
  EXPECT_ANY_THROW(SBRMP::fpsFromUnit(MTime::Unit::kHours));
  EXPECT_ANY_THROW(SBRMP::fpsFromUnit(MTime::Unit::kMinutes));
}

TEST_F(MayaUtilsTest, ScopedCurrentNamespace_empty) {
  MStatus status;
  {
    auto c = MNamespace::currentNamespace(&status);
    CHECK_MSTATUS(status);
    ASSERT_EQ(MString(":"), c);
  }
  {
    SBRMP::ScopedCurrentNamespace nm("");
    auto c = MNamespace::currentNamespace(&status);
    CHECK_MSTATUS(status);
    ASSERT_EQ(MString(":"), c);
  }
  {
    auto c = MNamespace::currentNamespace(&status);
    CHECK_MSTATUS(status);
    ASSERT_EQ(MString(":"), c);
  }
}

TEST_F(MayaUtilsTest, ScopedCurrentNamespace_basic) {
  MStatus status;
  {
    auto c = MNamespace::currentNamespace(&status);
    ASSERT_EQ(MString(":"), c);
    CHECK_MSTATUS(status);
  }
  {
    SBRMP::ScopedCurrentNamespace nm("a");
    {
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":a"), c);
    }
    {
      SBRMP::ScopedCurrentNamespace nm(":b");
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":b"), c);
    }
    {
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":a"), c);
    }
    {
      SBRMP::ScopedCurrentNamespace nm("b");
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":a:b"), c);
    }
    {
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":a"), c);
    }
    {
      // add a scoped namespce for the same namespace
      SBRMP::ScopedCurrentNamespace nm(":a");
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":a"), c);
    }
    {
      auto c = MNamespace::currentNamespace(&status);
      CHECK_MSTATUS(status);
      ASSERT_EQ(MString(":a"), c);
    }
  }
  {
    auto c = MNamespace::currentNamespace(&status);
    CHECK_MSTATUS(status);
    ASSERT_EQ(MString(":"), c);
  }
}

TEST_F(MayaUtilsTest, lockTransformPlugs) {
  MStatus status;
  MFnTransform transformFn;
  transformFn.create(MObject::kNullObj, &status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = SBRMP::lockTransformPlugs(transformFn);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
  // locking again is ok
  status = SBRMP::lockTransformPlugs(transformFn);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
}

TEST_F(MayaUtilsTest, lockIkJointPlugs) {
  MStatus status;
  MFnIkJoint ikJointFn;
  ikJointFn.create(MObject::kNullObj, &status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = SBRMP::lockIkJointPlugs(ikJointFn);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
  // locking again is ok
  status = SBRMP::lockIkJointPlugs(ikJointFn);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
}

TEST_F(MayaUtilsTest, unlockTransformPlug) {
  MStatus status;
  MFnIkJoint ikJointFn;
  ikJointFn.create(MObject::kNullObj, &status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = SBRMP::lockIkJointPlugs(ikJointFn);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());

  EXPECT_TRUE(SBRMP::getIkJointAnimatablePlugs(ikJointFn.object()).empty());

  // unlock an axis
  status = SBRMP::unlockTransformPlug(ikJointFn, SBRMP::ScrewAxis::rz);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
  auto plugs = SBRMP::getIkJointAnimatablePlugs(ikJointFn.object());
  EXPECT_EQ(1u, plugs.size());
  EXPECT_EQ(MString("rz"), plugs.at(0).partialName());

  // unlocking the same axis again is ok
  status = SBRMP::unlockTransformPlug(ikJointFn, SBRMP::ScrewAxis::rz);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());

  // unlock another axis
  status = SBRMP::unlockTransformPlug(ikJointFn, SBRMP::ScrewAxis::tx);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
  plugs = SBRMP::getIkJointAnimatablePlugs(ikJointFn.object());
  EXPECT_EQ(2u, plugs.size());
  EXPECT_EQ(MString("rz"), plugs.at(0).partialName());
  EXPECT_EQ(MString("tx"), plugs.at(1).partialName());
}

TEST_F(MayaUtilsTest, importColladaMesh_fail) {
  auto filename = MString((almodelutils_prefix_path + "/share/alrobotmodel/meshes/non_existing.dae").c_str());
  auto groupname = MString("mygroupname");
  MObject object;
  auto status = SBRMP::importColladaMesh(filename, groupname, object);
  EXPECT_NE(MStatus::kSuccess, status.statusCode());
  EXPECT_TRUE(object.isNull());
}

TEST_F(MayaUtilsTest, getIkJointsAnimatablePlugsMap) {
  MStatus status;
  MFnIkJoint ikJointFn;
  MObject legObj = ikJointFn.create();
  ikJointFn.setName("joint_Leg");
  status = SBRMP::lockTransformPlugs(ikJointFn);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = SBRMP::unlockTransformPlug(ikJointFn, SBRMP::ScrewAxis::rz);
  CHECK_MSTATUS_AND_THROW_IT(status);

  MObject torsoObj = ikJointFn.create(legObj);
  ikJointFn.setName("joint_Torso");
  // torso has no keyable translation nor rotation plug
  SBRMP::lockTransformPlugs(ikJointFn);

  MObject headObj = ikJointFn.create(torsoObj);
  ikJointFn.setName("joint_Head");
  status = SBRMP::lockTransformPlugs(ikJointFn);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = SBRMP::unlockTransformPlug(ikJointFn, SBRMP::ScrewAxis::rz);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = SBRMP::unlockTransformPlug(ikJointFn, SBRMP::ScrewAxis::tx);
  CHECK_MSTATUS_AND_THROW_IT(status);

  auto m0 = SBRMP::getIkJointsAnimatablePlugsMap();
  EXPECT_EQ(3u, m0.size());
  EXPECT_EQ(1u, m0.count("joint_Leg"));
  EXPECT_EQ(0u, m0.count("joint_Torso"));
  EXPECT_EQ(2u, m0.count("joint_Head"));

  auto m1 = SBRMP::getIkJointsAnimatablePlugsMap(legObj);
  EXPECT_EQ(m0, m1);

  auto m2 = SBRMP::getIkJointsAnimatablePlugsMap(torsoObj);
  EXPECT_EQ(2u, m2.size());
  EXPECT_EQ(0u, m2.count("joint_Leg"));
  EXPECT_EQ(0u, m2.count("joint_Torso"));
  EXPECT_EQ(2u, m2.count("joint_Head"));
}

// disable because it does not work from test (aka. library application),
// but it does work from the Maya plugin.
TEST_F(MayaUtilsTest, DISABLED_importColladaMesh_success) {
  auto filename = MString((collada_mesh_path).c_str());
  {
    // before getting further, check the file really exists
    auto fin = std::ifstream(filename.asChar());
    char buf[1024];
    auto n = fin.readsome(buf, 1024);
    ASSERT_TRUE(fin.good())
      << "could not read from file \""
      << filename.asChar() << "\"";
  }

  auto groupname = MString("mygroupname");
  MObject object;
  auto status = SBRMP::importColladaMesh(filename, groupname, object);
  EXPECT_EQ(MStatus::kSuccess, status.statusCode());
  EXPECT_FALSE(object.isNull());
  MFnTransform transformFn(object);
  EXPECT_EQ(groupname, transformFn.name());
  // TODO: check there is a mesh
}

class MayaUrdfTest : public MayaTest {};

TEST_F(MayaUrdfTest, toMayaIkJoint) {
  ptree joint;
  joint.put("<xmlattr>.type", "revolute");
  joint.put("origin.<xmlattr>.xyz", "0.1 0.2 0.3");
  joint.put("origin.<xmlattr>.rpy", "1.5708 0 0");
  joint.put("axis.<xmlattr>.xyz", "0 0 1");
  MFnIkJoint ikJointFn(SBRMP::toMayaIkJoint(joint));
  // todo: check the transform matrix
  // todo: check the attributes are locked
  // todo: check joint limits
}

TEST_F(MayaUrdfTest, toMayaTransform) {
  ptree joint;
  joint.put("<xmlattr>.type", "fixed");
  joint.put("origin.<xmlattr>.xyz", "0.1 0.2 0.3");
  joint.put("origin.<xmlattr>.rpy", "1.5708 0 0");
  MFnTransform ikJointFn(SBRMP::toMayaTransform(joint));
  // todo: check the transform matrix
  // todo: check the attributes are locked
}

TEST_F(MayaUrdfTest, findExistingFileFromUri) {
  // unsupported URI schemes
  EXPECT_ANY_THROW(SBRMP::findExistingFileFromUrdfUri("http://hello"));
  EXPECT_ANY_THROW(SBRMP::findExistingFileFromUrdfUri("package://"));
  EXPECT_ANY_THROW(SBRMP::findExistingFileFromUrdfUri("file://localhost/" + collada_mesh_path));
  EXPECT_NO_THROW(SBRMP::findExistingFileFromUrdfUri("file:///" + collada_mesh_path));
  // Note: the test below fails because the returned path is canonized:
  //EXPECT_EQ(MString(collada_mesh_path.c_str()),
  //          SBRMP::findExistingFileFromUrdfUri("file:///" + collada_mesh_path));
}

class MayaAnimTest : public MayaTest {};

// small helpers

ptree &require_key(ptree &ac, int frame, double value,
                   qianim::Side side, double tgt_abs, double tgt_ord) {
  auto &k = qianim::V2::ActuatorCurve::require_key(ac, frame);
  qianim::V2::Key::put_value(k, value);
  qianim::V2::Key::put_tangent(k, side, tgt_abs, tgt_ord);
  return k;
}

ptree &require_key(ptree &ac, int frame, double value,
                   double left_tgt_abs, double left_tgt_ord,
                   double right_tgt_abs, double right_tgt_ord) {
  auto &k = require_key(ac, frame, value, qianim::Side::left, left_tgt_abs, left_tgt_ord);
  qianim::V2::Key::put_tangent(k, qianim::Side::right, right_tgt_abs, right_tgt_ord);
  return k;
}

#ifdef WITH_ALMATHINTERNAL
using Frame = int;
using Key = AL::Math::Interpolation::Key;
using BezierKeysVector = std::vector<std::pair<const Frame, Key>>;
using BezierKeysInterpolation =
  AL::Math::Interpolation::BezierKeysInterpolation<
  BezierKeysVector::const_iterator>;

AL::Math::Position2D getTangent(const AL::qianim::ptree &key,
                                AL::qianim::Side side) {
  const auto &tangent = AL::qianim::V2::Key::get_tangent(key, side);
  return AL::Math::Position2D(
    AL::qianim::V2::Tangent::get_abscissa<float>(tangent),
    AL::qianim::V2::Tangent::get_ordinate<float>(tangent));
}

BezierKeysVector toBezierKeysVector(ptree &ac) {
  std::vector<std::pair<const Frame, Key>> seq;
  auto keys = AL::qianim::V2::ActuatorCurve::get_keys(ac);
  auto first = begin(keys);
  auto last = end(keys);

  if (first == last)
    return seq;
  seq.emplace_back(AL::qianim::V2::Key::get_frame(*first),
                   Key(AL::qianim::V2::Key::get_value<float>(*first)));
  auto adder = [&seq](
    const ptree &p0_key, const ptree &p3_key) mutable {
    AL::qianim::V2::Key::apply_cubic_bezier<float>(p0_key, p3_key,
      AL::qianim::V2::Key::check_cubic_bezier<float>);
    // add p1 as p0 right tangent
    seq.back().second.fRightTangent.fOffset
      = getTangent(p0_key, AL::qianim::Side::right);
    // add p3 key
    seq.emplace_back(AL::qianim::V2::Key::get_frame(p3_key),
      Key(AL::qianim::V2::Key::get_value<float>(p3_key)));
    // add p2 as p3 left tangent
    seq.back().second.fLeftTangent.fOffset
      = getTangent(p3_key, AL::qianim::Side::left);
  };
  AL::qianim::V2::adjacent_for_each(first, last, adder);
  return seq;
}

TEST_F(MayaAnimTest, toMayaAnimActuatorCurve_toQiAnimActuatorCurve) {
  ptree qiac;
  auto fps = 25;
  auto unit = MTime::Unit::kPALFrame; // 25 fps
  auto k = boost::math::constants::pi<float>() / 180;
  qianim::V2::ActuatorCurve::put_fps(qiac, fps);
  qianim::V2::ActuatorCurve::put_unit(qiac, qianim::Unit::degree);

  require_key(qiac, 0, 5., qianim::Side::right, 1.05, 2.1);
  require_key(qiac, 12, 5., -1.1, 2.2, 1.2, -2.4);
  require_key(qiac, 24, -5., qianim::Side::left, -2., 0.2);

  auto acObj = SBRMP::toMayaAnimCurve(qiac);
  MFnAnimCurve acFn(acObj);

  auto seq = toBezierKeysVector(qiac);
  BezierKeysInterpolation qifun(seq.begin(), seq.end());
  MStatus status;
  EXPECT_EQ(3u, acFn.numKeys());
  for (auto f = 0; f <= 24; ++f) {
    auto time = MTime(f, unit);
    auto y = static_cast<float>(acFn.evaluate(time, &status));
    CHECK_MSTATUS_AND_THROW_IT(status);
    // Maya uses radian
    EXPECT_NEAR(k * qifun(1.f * f), y, 1e-5f);
  }
  auto start = MTime(0.);
  auto end = MTime(24, unit);
  auto offset = MTime(0.);

  ptree qiacb = SBRMP::toQiAnimActuatorCurve(acFn, 2 * fps, start, end,
                                             offset);
  auto seqb = toBezierKeysVector(qiacb);
  BezierKeysInterpolation qifunb(seqb.begin(), seqb.end());

  for (auto f = 0; f <= 24; ++f) {
    CHECK_MSTATUS_AND_THROW_IT(status);
    // qiacb (anf qifunb) uses radian
    EXPECT_NEAR(k * qifun(1.f * f), qifunb(2.f * f), 1e-5f);
  }
}
#endif

TEST_F(MayaAnimTest, appendQiAnimActuatorCurve) {
  ptree ac;
  qianim::V2::ActuatorCurve::put_fps(ac, 25);
  qianim::V2::ActuatorCurve::put_unit(ac, qianim::Unit::radian);

  require_key(ac, 0, 5., qianim::Side::right, 1.05, 2.1);
  require_key(ac, 12, 5., -1.1, 2.2, 1.2,-2.4);
  require_key(ac, 24, -5., qianim::Side::left, -2., 0.2);
  MTime duration(24., MTime::Unit::kPALFrame);
  MTime offset(3., MTime::Unit::kSeconds);
  auto coeff = 3.14;
  MFnAnimCurve acFn;
  acFn.create(MFnAnimCurve::kAnimCurveTA);
  auto n = SBRMP::appendQiAnimActuatorCurve(ac, acFn, offset);
  EXPECT_EQ(3u, acFn.numKeys());
  EXPECT_EQ(3u, n);

  EXPECT_ANY_THROW(SBRMP::appendQiAnimActuatorCurve(ac, acFn));

  EXPECT_ANY_THROW(SBRMP::appendQiAnimActuatorCurve(ac, acFn,
                                                    offset + duration));

  n = SBRMP::appendQiAnimActuatorCurve(ac, acFn,
      offset + duration + MTime(0.001, MTime::Unit::kSeconds));
  EXPECT_EQ(6u, acFn.numKeys());
  EXPECT_EQ(6u, n);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  if (argc != 2) {
    std::cerr << "usage: test_mayaplugin <almodelutils_prefix_path>\n";
    return 1;
  }
  almodelutils_prefix_path = argv[1];
  collada_mesh_path = almodelutils_prefix_path + "/share/alrobotmodel/meshes/juliette/KneePitch.dae";
  MStatus status;
  status = MLibrary::initialize(argv[0], true);
  if (!status) {
    status.perror("MLibrary::initialize");
    return 1;
  }
  defaultUIUnit = MTime::uiUnit();
  CHECK_MSTATUS(status);

  auto ret = RUN_ALL_TESTS();
  MLibrary::cleanup(ret);
  return ret;
}
