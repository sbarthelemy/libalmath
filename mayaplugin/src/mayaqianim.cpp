#include "mayaqianim.h"
#include "utils.h"
#include <maya/MItDag.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MSelectionList.h>
#include <boost/math/constants/constants.hpp>

namespace qianim = AL::qianim;

namespace SBRMP {

// return MFnAnimCurve::AnimCurveType type and coeff such that if
//
//  maya_curve_type, coeff = toAnimCurveTypeAndScale(qianim_unit)
//
// then
//
//   value in maya_curve_unit = coeff * value in qianim_unit
std::pair<MFnAnimCurve::AnimCurveType, double> toAnimCurveTypeAndScale(qianim::Unit unit) {
  using Pair = std::pair<MFnAnimCurve::AnimCurveType, double>;
  switch (unit) {
  case qianim::Unit::degree:
    return Pair(MFnAnimCurve::kAnimCurveTA,
                boost::math::constants::pi<double>()/180);
  case qianim::Unit::radian:
    return Pair(MFnAnimCurve::kAnimCurveTA, 1.);
  case qianim::Unit::meter:
    return Pair(MFnAnimCurve::kAnimCurveTL, 100.);
  case qianim::Unit::dimensionless:
    return Pair(MFnAnimCurve::kAnimCurveTU, 1.);
  }
  std::stringstream msg;
  msg << "this qianim::Unit is not compatible with a "
    "MFnAnimCurve::AnimCurveType.";
  throw std::invalid_argument(msg.str());
}

MFnAnimCurve::AnimCurveType toAnimCurveType(qianim::Unit unit) {
  return toAnimCurveTypeAndScale(unit).first;
}

double toAnimCurveScale(qianim::Unit unit) {
  return toAnimCurveTypeAndScale(unit).second;
}

// return qianim unit and coeff such that if
//
//  qianim_unit, coeff = qiAnimValueConversion(maya_curve_type)
//
// then
//
//   value in qianim_unit = coeff * value in maya_curve_unit
std::pair<qianim::Unit, double> qiAnimValueConversion(
  MFnAnimCurve::AnimCurveType type) {
  using Pair = std::pair<qianim::Unit, double>;
  switch (type) {
  case MFnAnimCurve::kAnimCurveTA:
    return Pair(qianim::Unit::radian, 1.); // maya uses radians
  case MFnAnimCurve::kAnimCurveTL:
    return Pair(qianim::Unit::meter, 0.01); // maya uses centimeters
  case MFnAnimCurve::kAnimCurveTU:
    return Pair(qianim::Unit::dimensionless, 1.);
  case MFnAnimCurve::kAnimCurveTT:
  case MFnAnimCurve::kAnimCurveUA:
  case MFnAnimCurve::kAnimCurveUL:
  case MFnAnimCurve::kAnimCurveUT:
  case MFnAnimCurve::kAnimCurveUU:
  case MFnAnimCurve::kAnimCurveUnknown:
    break;
  }
  std::stringstream msg;
  msg << "AnimCurveType " << type << " is not compatible with qianim::Unit.";
  throw std::invalid_argument(msg.str());
}

void _setTangent(MFnAnimCurve &acFn, unsigned int keyIndex, const ptree &key,
  qianim::Side side, int fps, float coeff_) {
  assert(fps > 0);
  const ptree & tangent = qianim::V2::Key::get_tangent(key, side);
  const auto x = qianim::V2::Tangent::get_abscissa<float>(tangent);
  const auto y = qianim::V2::Tangent::get_ordinate<float>(tangent);
  const auto in = (side == qianim::Side::left);
  const auto s = (in ? -3.f : 3.f);
  const auto convertUnits = false;
  const auto status = acFn.setTangent(keyIndex, s / fps * x, s * coeff_ * y, in,
    NULL, convertUnits);
  CHECK_MSTATUS_AND_THROW_IT(status);
};

unsigned int appendQiAnimActuatorCurve(const AL::qianim::ptree &curve,
                                       MFnAnimCurve &animCurveFn,
                                       MTime offset) {
  MStatus status;
  auto type = animCurveFn.animCurveType(&status);
  if (status != MStatus::kSuccess)
    throw std::runtime_error("animCurveFn is not attached to an MObject");
  auto qiunit = qianim::V2::ActuatorCurve::get_unit(curve);
  if (type != toAnimCurveType(qiunit))
    throw std::runtime_error("animCurveFn has an incompatible AnimCurveType");
  auto numKeys = animCurveFn.numKeys(&status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  const auto fps = qianim::V2::ActuatorCurve::get_fps(curve);
  const auto timeUnit = unitFromFps(fps);
  const auto coeff = toAnimCurveScale(qiunit);
  const auto coeff_ = static_cast<float>(coeff);
  auto keys = qianim::V2::ActuatorCurve::get_keys(curve);
  auto first = begin(keys);
  auto last = end(keys);
  if (first == last)
    return numKeys;


  // deal with the first key
  const auto time = offset +
      MTime(static_cast<double>(qianim::V2::Key::get_frame(*first)), timeUnit);
  // check this is an append operation
  if ((numKeys > 0u) && (time <= animCurveFn.time(numKeys - 1u))) {
    throw std::runtime_error("invalid time, would not be an append operation");
  }
  auto lastKeyIdx = animCurveFn.addKey(time,
    qianim::V2::Key::get_value<double>(*first) * coeff,
    MFnAnimCurve::TangentType::kTangentFixed,
    MFnAnimCurve::TangentType::kTangentFixed,
    NULL,
    &status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  status = animCurveFn.setTangentsLocked(lastKeyIdx, false);
  CHECK_MSTATUS_AND_THROW_IT(status);

  // a functor
  auto adder = [&lastKeyIdx, &animCurveFn, offset, timeUnit, fps, coeff,
    coeff_](
        const ptree &p0_key, const ptree &p3_key) mutable {
    // check the keys are valid
    qianim::V2::Key::apply_cubic_bezier<float>(p0_key, p3_key,
      qianim::V2::Key::check_cubic_bezier<float>);
    // add p0_key's right tangent (the key was already added to animCurveFn)
    _setTangent(animCurveFn, lastKeyIdx, p0_key, qianim::Side::right, fps, coeff_);
    // add p3_key
    const auto time = offset +
        MTime(static_cast<double>(qianim::V2::Key::get_frame(p3_key)),
          timeUnit);
    MStatus status;
    lastKeyIdx = animCurveFn.addKey(time,
      qianim::V2::Key::get_value<double>(p3_key) * coeff,
      MFnAnimCurve::TangentType::kTangentFixed,
      MFnAnimCurve::TangentType::kTangentFixed,
      NULL,
      &status);
    CHECK_MSTATUS_AND_THROW_IT(status);
    status = animCurveFn.setTangentsLocked(lastKeyIdx, false);
    CHECK_MSTATUS_AND_THROW_IT(status);
    // add p3_key's left tangent
    _setTangent(animCurveFn, lastKeyIdx, p3_key, qianim::Side::left, fps, coeff_);
  };

  // add all the keys, except the first
  qianim::V2::adjacent_for_each(first, last, adder);
  return lastKeyIdx + 1u;
}

MObject toMayaAnimCurve(const AL::qianim::ptree &curve, MTime offset) {
  auto type = toAnimCurveType(qianim::V2::ActuatorCurve::get_unit(curve));
  MFnAnimCurve animCurveFn;
  MStatus status;
  auto obj = animCurveFn.create(type, nullptr, &status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  appendQiAnimActuatorCurve(curve, animCurveFn, offset);
  return obj;
}

MTime importQiAnimation(const PlugsMap &plugsMap,
                        const AL::qianim::ptree &animation,
                        MTime offset) {
  MTime endTime(0.);
  MStatus status;
  // iterate over the qianim curves.
  // Find or create the corresponding MFnAnimCurve,
  // and append to it.
  for (const auto &curve :
       qianim::V2::Animation::get_actuatorcurves(animation)) {
    try {
      if (qianim::V2::ActuatorCurve::get_mute(curve))
        continue;
      const auto actuator = qianim::V2::ActuatorCurve::get_actuator(curve);
      auto plugIt = plugsMap.find(actuator);
      if (plugIt == plugsMap.end())
        continue;
      MObject obj;
      // Note: plugMap may contain several animatable plugs under the
      // `actuator` key. We only consider the first one, since SBR robots
      // only have a single degree of freedom per joint
      MPlug plug = plugIt->second;
      if (plug.isNull())
        continue;
      if (!plug.isConnected()) {
        auto type = toAnimCurveType(qianim::V2::ActuatorCurve::get_unit(curve));
        MFnAnimCurve animCurveFn;
        obj = animCurveFn.create(plug, type, nullptr, &status);
        CHECK_MSTATUS_AND_THROW_IT(status);
      } else {
        MFnAnimCurve animCurveFn(plug, &status);
        obj = animCurveFn.object();
        CHECK_MSTATUS_AND_THROW_IT(status);
      }
      assert(!obj.isNull());
      MFnAnimCurve animCurveFn(obj);
      auto n = appendQiAnimActuatorCurve(curve, animCurveFn, offset);
      endTime = std::max(endTime, animCurveFn.time(n - 1u));
    } catch (...) {
      continue;
    }
  }
  return endTime;
}

ptree toQiAnimActuatorCurve(const MFnAnimCurve &acFn, const int fps,
  const MTime start, const MTime end, const MTime offset) {
  MStatus status;

  auto valueUnit = qianim::Unit::degree;
  auto valueCoeff = 1.;

  const auto type = acFn.animCurveType(&status);
  CHECK_MSTATUS_AND_THROW_IT(status);
  std::tie(valueUnit, valueCoeff) = qiAnimValueConversion(type);

  auto timeUnit = MTime::k6000FPS;
  auto timeCoeff = fps / 6000.;
  try {
    timeUnit = unitFromFps(fps);
    timeCoeff = 1.;
  } catch (...) {}

  ptree ac;
  qianim::V2::ActuatorCurve::put_fps(ac, fps);
  qianim::V2::ActuatorCurve::put_unit(ac, valueUnit);

  const auto thirdOfValueCoeff = static_cast<float>(valueCoeff / 3);
  float x = 0.f, y = 0.f;
  const float thirdOfFPS = fps / 3.f;

  for (unsigned int k = 0; k < acFn.numKeys(); k++) {
    auto t = acFn.time(k);
    if (t < start)
      continue;
    if (t > end)
      break;
    const auto ajustedTime = t - start + offset;
    const auto frame = static_cast<int>(
      std::lround(timeCoeff * ajustedTime.asUnits(timeUnit)));
    auto &key = qianim::V2::ActuatorCurve::require_key(ac, frame);
    qianim::V2::Key::put_value(key, valueCoeff * acFn.value(k));
    {
      status = acFn.getTangent(k, x, y, true);
      CHECK_MSTATUS_AND_THROW_IT(status);
      auto &tgt = qianim::V2::Key::put_tangent(key, qianim::Side::left,
        -thirdOfFPS * x, -thirdOfValueCoeff * y);
    }
    {
      status = acFn.getTangent(k, x, y, false);
      CHECK_MSTATUS_AND_THROW_IT(status);
      auto &tgt = qianim::V2::Key::put_tangent(key, qianim::Side::right,
        thirdOfFPS * x, thirdOfValueCoeff * y);
    }
  }
  return ac;
}

AL::qianim::ptree exportQiAnimation(const PlugsMap &plugsMap, int fps,
                                    MTime start, MTime end,
                                    MTime offset) {
  AL::qianim::ptree anim;
  anim.put("<xmlattr>.typeVersion", "2.0");
  for (auto it = plugsMap.begin(); it != plugsMap.end(); ++it) {
    if (!it->second.isConnected())
      continue;
    MFnAnimCurve ac(it->second);
    auto &c = anim.add_child("ActuatorCurve",
                             toQiAnimActuatorCurve(ac, fps, start, end,
                                                   offset));
    AL::qianim::V2::ActuatorCurve::put_actuator(c, it->first);
  }
  return anim;
}
}
