#ifndef SOFTBANKROBOTICSMAYAPLUGIN_SRC_MAYAQIANIM_H
#define SOFTBANKROBOTICSMAYAPLUGIN_SRC_MAYAQIANIM_H

#include <maya/MObject.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MTime.h>
#include <almath/scenegraph/qianim.h>
#include "utils.h"

namespace SBRMP {

// insert the keys from curve into animCurveFn.
// A key a time "t" in curve will be inserted at time "timeOffset + t"
// in animCurveFn.
//
// Throw if the curves have incompatible units/types
// Throw if keys to insert are not at the end of animCurveFn
// (we want to append keys, not to insert them).
//
// Return the total number of keys in animCurveFn.
unsigned int appendQiAnimActuatorCurve(const AL::qianim::ptree &curve,
                                       MFnAnimCurve &animCurveFn,
                                       MTime offset = MTime(0.));

MObject toMayaAnimCurve(const AL::qianim::ptree &curve,
                        MTime offset = MTime(0.));

// For each ActuatorCurve in the URDF animation, search the scene
// (below root) for an IkJoint with the same name,
// then for an animatable plug, and append the ActuatorCurve to it.
//
// An key at time `t` in the qi animation will be inserted at time
// `t + offset` in the Maya AnimCurve.
//
// Curves with wrong type or for which keys to insert are not at the
// end of the AbimCurve are skipped.
//
// Return the time of the last key of all the touched curves.
MTime importQiAnimation(const PlugsMap &plugsMap,
                        const AL::qianim::ptree &animation,
                        MTime offset = MTime(0.));

// Create an ActuatorCurve element from the Maya AnimCurve
// The ActuatorCurve name attribute is *not* set.
// If needed, the key frames will be rounded to match the requested fps.
//
// Only the Maya keys at time `t` such that `start <= t <= end `
// are exported.
// The key will then be at time  `t - start + offset` in the qi animation.
AL::qianim::ptree toQiAnimActuatorCurve(const MFnAnimCurve &acFn,
                                        int fps,
                                        MTime start,
                                        MTime end,
                                        MTime offset = MTime(0.));

// Return a qi Animation element containing one ActuatorCurve
// for each AnimCurve attached to a plug from plugsMap.
//
// The ActuatorCurves are named after the IKJoint.
//
// Only the Maya keys at time `t` such that `start <= t <= end`
// are exported.
// The key will then be at time  `t - start + offset` in the qi animation.
AL::qianim::ptree exportQiAnimation(const PlugsMap &plugsMap, int fps,
                                    MTime start, MTime end,
                                    MTime offset = MTime(0.));
}
#endif