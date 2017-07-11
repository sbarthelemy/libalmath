#include <maya/MFnPlugin.h>
#include <maya/MPxCommand.h>
#include <maya/MIOStream.h>

#include <maya/MAnimControl.h>
#include <maya/MArgList.h>
#include "utils.h"
#include "mayaurdf.h"
#include "mayaqianim.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <regex>

namespace {
ptree read_xml(const MString &filename) {
  ptree root;
  auto is = SBRMP::create_ifstream(filename);
  boost::property_tree::xml_parser::read_xml(is, root,
    boost::property_tree::xml_parser::trim_whitespace);
  return root;
}
}

// load a urdf file
class LoadUrdfCommand : public MPxCommand {
private:
  bool withSbrPreProcessing;
public:

  static bool asSbrIkJoint(const ptree &joint) {
    // match if
    // NOT <chainname>_fixedjoint
    // AND (ends with _fixedjoint OR is <wheelname>)
    const std::regex re(
        "(?!((R|L)?Leg|(R|L)Arm)_fixedjoint)(.*_fixedjoint|Wheel(FL|FR|B))$");
    return !std::regex_match(AL::urdf::name(joint), re) &&
           !(AL::urdf::Joint(joint).mimic());
  }

  LoadUrdfCommand(bool withSbrPreProcessing)
    : withSbrPreProcessing(withSbrPreProcessing) {}

  MStatus doIt(const MArgList &arglist) {
    MStatus status;
    // will fail if arglist.length() < 1
    auto filename = arglist.asString(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    ptree root = ::read_xml(filename);
    ptree &robot = root.get_child("robot");
    AL::urdf::RobotTree rtree(robot);
    if (arglist.length() > 1) {
      if (arglist.length() != 3)
        return MStatus::kFailure;
      auto meshPrefixFrom = arglist.asString(1, &status);
      CHECK_MSTATUS_AND_RETURN_IT(status);
      auto meshPrefixTo = arglist.asString(2, &status);
      CHECK_MSTATUS_AND_RETURN_IT(status);

      // change the filenames referenced in the urdf:
      // the paths prefixed by meshPrefixFrom should now be
      // prefixed by meshPrefixTo.
      auto op = [meshPrefixFrom, meshPrefixTo](std::string in) {
        return std::regex_replace(
            in,
            std::regex(std::string("^") + meshPrefixFrom.asChar()),
            meshPrefixTo.asChar());
        };
        AL::urdf::robot::transform_filenames(robot, op);
    }
    auto disableSelectionChildHighlighting = true;
    auto importVisuals = true;
    auto hideIkJoints = true;
    MFnTransform rootTransformFn;
    auto rootTransformObj = rootTransformFn.create(MObject::kNullObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    rootTransformFn.setName(AL::urdf::name(robot).c_str(), false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    if (withSbrPreProcessing) {
      if (rtree.root_link() == "base_link") {
        // remove the phony "base_link_fixedjoint"
        rtree.rm_root_joint();
      }
      if (true) {// change referenced meshes extension from .mesh to .dae
        auto op = [](std::string in) -> std::string {
          return std::regex_replace(in, std::regex("\\.mesh$"), ".dae");
        };
        AL::urdf::robot::transform_filenames(robot, op);
      }
      const auto name = AL::urdf::name(robot);
      if (boost::algorithm::istarts_with(name, "juliette")) {
          // each wheel has one degree of freedom, but we don't want to
          // animate them, so let convert them to fixed joints.
          makeJointFixed(rtree, "WheelB");
          makeJointFixed(rtree, "WheelFL");
          makeJointFixed(rtree, "WheelFR");
          // The URDF root link is the Torso, but the animation is easier
          // starting from the Leg.
          rtree.define_as_root_link("Leg");
      }
    }
    auto index = SBRMP::importUrdfRobotJoints(rtree,
        withSbrPreProcessing ? asSbrIkJoint : SBRMP::returnTrue,
        rootTransformObj);
    if (disableSelectionChildHighlighting) {
       status = SBRMP::setAttr<bool>(rootTransformObj,
                                         "selectionChildHighlighting", false);
      CHECK_MSTATUS_AND_RETURN_IT(status);
      }

    if (hideIkJoints) {
      MFnIkJoint ikJointFn;
      for (auto it = index.links.begin(); it != index.links.end(); it++) {
        if (it->second.hasFn(MFn::kJoint)) {
          status = SBRMP::setAttr<int>(it->second, "drawStyle", 2);
          CHECK_MSTATUS_AND_RETURN_IT(status);
        }
      }
    }
    if (importVisuals) {
      SBRMP::ScopedCurrentNamespace currentNamespace("visual");
      SBRMP::importUrdfVisuals(rtree, index);
    }
    return status;
  }

  static void *createSbr() {
    return new LoadUrdfCommand(true);
  }

  static void *createRaw() {
    return new LoadUrdfCommand(false);
  }
};

class AddControllers : public MPxCommand {
public:
  static void *create() {
    return new AddControllers();
  }
  MStatus doIt(const MArgList &arglist) {
    MObject root = MObject::kNullObj; //< todo get a name param or so
    auto plugsMap = SBRMP::getIkJointsAnimatablePlugsMap(root);
    auto radius = 10.; // in cm
    SBRMP::addControllers(plugsMap, radius);
    return MStatus::kSuccess;
  }
};

// Import a qianim file (animation file)
class QiAnimImport  : public MPxCommand {
public:
  QiAnimImport(){}

  MStatus doIt(const MArgList &arglist) {
    MStatus status;
    // will fail if arglist.length() < 1
    auto filename = arglist.asString(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    if (arglist.length() != 1)
      MStatus::kFailure;
    AL::qianim::ptree root = ::read_xml(filename);
    const AL::qianim::ptree &animation = AL::qianim::V2::get_animation(root);
    auto plugsMap = SBRMP::getIkJointsAnimatablePlugsMap();
    auto offset = SBRMP::maxAnimCurveTime(plugsMap) + MTime(1.);
    MTime lastKeyTime = SBRMP::importQiAnimation(plugsMap, animation, offset);

    // Set the current maxTime of the Maya timeline to the last global
    // key time read in qianim
    MAnimControl ac;
    return ac.setMaxTime(lastKeyTime);
  }

  static void *create() {
    return new QiAnimImport();
  }

};

// Export a qianim file from the current Maya scene
class QiAnimExport : public MPxCommand {
public:
  QiAnimExport(){}

  // TODO : Create a qianim file from a Maya scene
  MStatus doIt(const MArgList &arglist) {
    MStatus status;
    try {
      MAnimControl actrl;
      AL::qianim::ptree xmlroot;
      //xmlroot.put("<xmlattr>.xmlns:editor", "");
      xmlroot.add_child("Animation",
          SBRMP::exportQiAnimation(SBRMP::getIkJointsAnimatablePlugsMap(),
                                   SBRMP::fpsFromUnit(MTime::uiUnit()),
                                   actrl.minTime(), actrl.maxTime(),
                                   MTime(1.0)));
      // will fail if arglist.length() < 1
      auto filename = arglist.asString(0, &status);
      CHECK_MSTATUS_AND_RETURN_IT(status);

      std::ofstream of = SBRMP::create_ofstream(filename);
      boost::property_tree::xml_writer_settings<std::string> settings('\t', 1);
      boost::property_tree::xml_parser::write_xml(of, xmlroot, settings);
      return status;
    } catch (const std::exception &e) {
      MPxCommand::displayError(e.what());
      return MStatus::kFailure;
    }
  }

  static void *create() {
    return new QiAnimExport();
  }
};

MStatus initializePlugin(MObject obj) {
  MFnPlugin pluginFn(obj, "SoftBank Robotics", SOFTBANKROBOTICSMAYAPLUGIN_VERSION);
  MStatus status;
  // load a urdf file, with custom preprocessing specific to SBR robots.
  // The nature of the preprocessing depends on the robot name.
  status = pluginFn.registerCommand("loadSbrUrdf", LoadUrdfCommand::createSbr);
  CHECK_MSTATUS_AND_RETURN_IT(status);
  // load a urdf file, without preprocessing
  status = pluginFn.registerCommand("loadRawUrdf", LoadUrdfCommand::createRaw);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  // Add Controllers to an existing IKJoint tree
  status = pluginFn.registerCommand("addControllers", AddControllers::create);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  // Import qianim file
  status = pluginFn.registerCommand("importQiAnim", QiAnimImport::create);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  // Export qianim file
  status = pluginFn.registerCommand("exportQiAnim", QiAnimExport::create);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  return status;
}

MStatus uninitializePlugin(MObject obj) {
  MFnPlugin pluginFn(obj);
  MStatus status;
  status = pluginFn.deregisterCommand("loadRawUrdf");
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = pluginFn.deregisterCommand("loadSbrUrdf");
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = pluginFn.deregisterCommand("addControllers");
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = pluginFn.deregisterCommand("importQiAnim");
  CHECK_MSTATUS_AND_RETURN_IT(status);
  status = pluginFn.deregisterCommand("exportQiAnim");
  CHECK_MSTATUS_AND_RETURN_IT(status);
  return status;
}
