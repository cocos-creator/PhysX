#include "PxPhysicsAPI.h"
#include <chrono>
#include <ctime>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include "PxWebBindings.h"

using namespace physx;
using namespace emscripten;

#define __LIB_VERSION__ 100

struct PxRaycastCallbackWrapper : public wrapper<PxRaycastCallback> {
  EMSCRIPTEN_WRAPPER(PxRaycastCallbackWrapper)
  PxAgain processTouches(const PxRaycastHit *buffer, PxU32 nbHits) {
    for (PxU32 i = 0; i < nbHits; i++) {
      bool again = call<PxAgain>("processTouches", buffer[i]);
      if (!again) {
        return false;
      }
    }
    return true;
  }
};

PxRaycastHit *allocateRaycastHitBuffers(PxU32 nb) {
  PxRaycastHit *myArray = new PxRaycastHit[nb];
  return myArray;
}

struct PxSweepCallbackWrapper : public wrapper<PxSweepCallback> {
  EMSCRIPTEN_WRAPPER(PxSweepCallbackWrapper)
  PxAgain processTouches(const PxSweepHit *buffer, PxU32 nbHits) {
    for (PxU32 i = 0; i < nbHits; i++) {
      bool again = call<PxAgain>("processTouches", buffer[i]);
      if (!again) {
        return false;
      }
    }
    return true;
  }
};

PxSweepHit *allocateSweepHitBuffers(PxU32 nb) {
  PxSweepHit *myArray = new PxSweepHit[nb];
  return myArray;
}

struct PxQueryFilterCallbackWrapper : public wrapper<PxQueryFilterCallback> {
  EMSCRIPTEN_WRAPPER(PxQueryFilterCallbackWrapper)
  PxQueryHitType::Enum postFilter(const PxFilterData &filterData,
                                  const PxQueryHit &hit) {
    return call<PxQueryHitType::Enum>("postFilter", filterData, hit);
  }
  PxQueryHitType::Enum preFilter(const PxFilterData &filterData,
                                 const PxShape *shape,
                                 const PxRigidActor *actor, PxHitFlags &out) {
    // // group mask filter
    // const PxFilterData &fd1 = shape->getQueryFilterData();
    // if (!(filterData.word1 & fd1.word0))
    // {
    //   return PxQueryHitType::eNONE;
    // }
    PxQueryHitType::Enum hitType =
        call<PxQueryHitType::Enum>("preFilter", filterData, shape, actor, out);
    return hitType;
  }
};

bool gContactPointsNeedClear = false;
std::vector<PxContactPairPoint> gContactPoints;
std::vector<PxContactPairPoint> getGContacts() { return gContactPoints; }
struct PxSimulationEventCallbackWrapper
    : public wrapper<PxSimulationEventCallback> {
  EMSCRIPTEN_WRAPPER(PxSimulationEventCallbackWrapper)
  void onConstraintBreak(PxConstraintInfo *, PxU32) {}
  void onWake(PxActor **, PxU32) {}
  void onSleep(PxActor **, PxU32) {}
  void onContact(const PxContactPairHeader &, const PxContactPair *pairs,
                 PxU32 nbPairs) {
    if (gContactPointsNeedClear) {
      gContactPoints.clear();
      gContactPointsNeedClear = false;
    }
    for (PxU32 i = 0; i < nbPairs; i++) {
      const PxContactPair &cp = pairs[i];

      if (cp.flags & (PxContactPairFlag::eREMOVED_SHAPE_0 |
                      PxContactPairFlag::eREMOVED_SHAPE_1))
        continue;

      std::vector<PxContactPairPoint> contactVec;
      const PxU8 &contactCount = cp.contactCount;
      const PxU32 offset = gContactPoints.size();
      if (contactCount) {
        contactVec.resize(contactCount);
        pairs[i].extractContacts(&contactVec[0], contactCount);
        gContactPoints.insert(gContactPoints.cend(), contactVec.cbegin(),
                              contactVec.cend());
      }

      if (cp.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS) {
        call<void>("onContactPersist", cp.shapes[0], cp.shapes[1], contactCount,
                   gContactPoints, offset);
      } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND) {
        call<void>("onContactBegin", cp.shapes[0], cp.shapes[1], contactCount,
                   gContactPoints, offset);
      } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_LOST) {
        call<void>("onContactEnd", cp.shapes[0], cp.shapes[1], contactCount,
                   gContactPoints, offset);
      }
    }
  }
  void onTrigger(PxTriggerPair *pairs, PxU32 count) {
    for (PxU32 i = 0; i < count; i++) {
      const PxTriggerPair &tp = pairs[i];
      if (tp.flags & (PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER |
                      PxTriggerPairFlag::eREMOVED_SHAPE_OTHER))
        continue;

      if (tp.status & PxPairFlag::eNOTIFY_TOUCH_FOUND) {
        call<void>("onTriggerBegin", tp.triggerShape, tp.otherShape,
                   tp.triggerActor, tp.otherActor);
      } else if (tp.status & PxPairFlag::eNOTIFY_TOUCH_LOST) {
        call<void>("onTriggerEnd", tp.triggerShape, tp.otherShape,
                   tp.triggerActor, tp.otherActor);
      }
      // Trigger do not support touch persists
      // else if (tp.status & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
      // {
      //   call<void>("onTriggerPersist", tp.triggerShape, tp.otherShape,
      //   tp.triggerActor, tp.otherActor);
      // }
    }
  }
  void onAdvance(const PxRigidBody *const *, const PxTransform *, const PxU32) {
  }
};

PxFilterFlags DefaultFilterShader(PxFilterObjectAttributes attributes0,
                                  PxFilterData fd0,
                                  PxFilterObjectAttributes attributes1,
                                  PxFilterData fd1, PxPairFlags &pairFlags,
                                  const void *, PxU32) {
  // constexpr PxU32 QUERY_FILTER = 1 << 0;
  // constexpr PxU32 QUERY_CHECK_TRIGGER = 1 << 1;
  // constexpr PxU32 QUERY_SINGLE_HIT = 1 << 2;
  constexpr PxU32 DETECT_TRIGGER_EVENT = 1 << 3;
  constexpr PxU32 DETECT_CONTACT_EVENT = 1 << 4;
  constexpr PxU32 DETECT_CONTACT_POINT = 1 << 5;
  constexpr PxU32 DETECT_CONTACT_CCD = 1 << 6;
  // group mask filter
  if (!(fd0.word0 & fd1.word1) || !(fd0.word1 & fd1.word0)) {
    return PxFilterFlag::eSUPPRESS;
  }

  pairFlags = PxPairFlags(0);

  // trigger filter
  if (PxFilterObjectIsTrigger(attributes0) ||
      PxFilterObjectIsTrigger(attributes1)) {
    pairFlags |= PxPairFlag::eDETECT_DISCRETE_CONTACT;

    // need trigger event?
    const PxU16 needTriggerEvent =
        (fd0.word3 & DETECT_TRIGGER_EVENT) | (fd1.word3 & DETECT_TRIGGER_EVENT);
    if (needTriggerEvent) {
      pairFlags |=
          PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_LOST;
      return PxFilterFlag::eDEFAULT;
    } else {
      return PxFilterFlag::eSUPPRESS;
    }
  }

  // need detect ccd contact?
  const PxU16 needDetectCCD =
      (fd0.word3 & DETECT_CONTACT_CCD) | (fd1.word3 & DETECT_CONTACT_CCD);
  if (needDetectCCD)
    pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;

  // simple collision process
  pairFlags |= PxPairFlag::eCONTACT_DEFAULT;

  // need contact event?
  const PxU16 needContactEvent =
      (fd0.word3 & DETECT_CONTACT_EVENT) | (fd1.word3 & DETECT_CONTACT_EVENT);
  if (needContactEvent)
    pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND |
                 PxPairFlag::eNOTIFY_TOUCH_LOST |
                 PxPairFlag::eNOTIFY_TOUCH_PERSISTS;

  // need contact point?
  const PxU16 needContactPoint =
      (fd0.word3 & DETECT_CONTACT_POINT) | (fd1.word3 & DETECT_CONTACT_POINT);
  if (needContactPoint)
    pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;

  return PxFilterFlag::eDEFAULT;
}

// TODO: Getting the  global PxDefaultSimulationFilterShader into javascript
// is problematic, so let's provide this custom factory function for now

PxSceneDesc *getDefaultSceneDesc(PxTolerancesScale &scale, int numThreads,
                                 PxSimulationEventCallback *callback) {
  PxSceneDesc *sceneDesc = new PxSceneDesc(scale);
  sceneDesc->gravity = PxVec3(0.0f, -9.81f, 0.0f);
  sceneDesc->cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
  sceneDesc->filterShader = DefaultFilterShader;
  sceneDesc->simulationEventCallback = callback;
  sceneDesc->kineKineFilteringMode = PxPairFilteringMode::eKEEP;
  sceneDesc->staticKineFilteringMode = PxPairFilteringMode::eKEEP;
  sceneDesc->flags |= PxSceneFlag::eENABLE_CCD;
  return sceneDesc;
}

PxConvexMesh *createConvexMesh(std::vector<PxVec3> &vertices,
                               PxCooking &cooking, PxPhysics &physics) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

  PxConvexMesh *convexMesh = cooking.createConvexMesh(
      convexDesc, physics.getPhysicsInsertionCallback());

  return convexMesh;
}

PxConvexMesh *createConvexMeshFromBuffer(int vertices, PxU32 vertCount,
                                         PxCooking &cooking,
                                         PxPhysics &physics) {
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertCount;
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = (PxVec3 *)vertices;
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

  PxConvexMesh *convexMesh = cooking.createConvexMesh(
      convexDesc, physics.getPhysicsInsertionCallback());

  return convexMesh;
}

PxTriangleMesh *createTriMesh(int vertices, PxU32 vertCount, int indices,
                              PxU32 indexCount, bool isU16, PxCooking &cooking,
                              PxPhysics &physics) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count = vertCount;
  meshDesc.points.stride = sizeof(PxVec3);
  meshDesc.points.data = (PxVec3 *)vertices;

  meshDesc.triangles.count = indexCount;
  if (isU16) {
    meshDesc.triangles.stride = 3 * sizeof(PxU16);
    meshDesc.triangles.data = (PxU16 *)indices;
    meshDesc.flags = PxMeshFlag::e16_BIT_INDICES;
  } else {
    meshDesc.triangles.stride = 3 * sizeof(PxU32);
    meshDesc.triangles.data = (PxU32 *)indices;
  }

  PxTriangleMesh *triangleMesh = cooking.createTriangleMesh(
      meshDesc, physics.getPhysicsInsertionCallback());
  return triangleMesh;
}

PxTriangleMesh *createTriMeshExt(std::vector<PxVec3> &vertices,
                                 std::vector<PxU16> &indices,
                                 PxCooking &cooking, PxPhysics &physics) {
  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count = vertices.size();
  meshDesc.points.stride = sizeof(PxVec3);
  meshDesc.points.data = (PxVec3 *)vertices.data();

  meshDesc.triangles.count = indices.size() / 3;
  meshDesc.triangles.stride = 3 * sizeof(PxU16);
  meshDesc.triangles.data = (PxU16 *)indices.data();
  meshDesc.flags = PxMeshFlag::e16_BIT_INDICES;

  PxTriangleMesh *triangleMesh = cooking.createTriangleMesh(
      meshDesc, physics.getPhysicsInsertionCallback());
  return triangleMesh;
}

PxHeightField *createHeightFieldExt(PxU32 numCols, PxU32 numRows,
                                    std::vector<PxHeightFieldSample> &samples,
                                    PxCooking &cooking, PxPhysics &physics) {
  PxHeightFieldDesc hfDesc;
  // hfDesc.format             = PxHeightFieldFormat::eS16_TM;
  hfDesc.nbColumns = numCols;
  hfDesc.nbRows = numRows;
  hfDesc.samples.data = samples.data();
  hfDesc.samples.stride = sizeof(PxHeightFieldSample);

  PxHeightField *heightField =
      cooking.createHeightField(hfDesc, physics.getPhysicsInsertionCallback());
  return heightField;
}

PxCapsuleController *
createCapsuleCharacterController(PxControllerManager &ctrlMgr,
                                 const PxCapsuleControllerDesc &desc) {
  PxController *ctrl = ctrlMgr.createController(desc);
  return dynamic_cast<PxCapsuleController *>(ctrl);
}

PxBoxController *createBoxCharacterController(PxControllerManager &ctrlMgr,
                                              const PxBoxControllerDesc &desc) {
  PxController *ctrl = ctrlMgr.createController(desc);
  return dynamic_cast<PxBoxController *>(ctrl);
}

struct PxUserControllerHitReportWrapper
    : public wrapper<PxUserControllerHitReport> {
  EMSCRIPTEN_WRAPPER(PxUserControllerHitReportWrapper)
  void onShapeHit(const PxControllerShapeHit &hit) {
    return call<void>("onShapeHit", hit);
  }
  void onControllerHit(const PxControllersHit &hit) {
    return call<void>("onControllerHit", hit);
  }
  void onObstacleHit(const PxControllerObstacleHit &hit) {
    return call<void>("onObstacleHit", hit);
  }
};

// PxRenderBuffer
static uint32_t PxRenderBuffer_GetNbLines(uint32_t ptr) {
  return ((PxRenderBuffer *)ptr)->getNbLines();
}
static uint32_t PxRenderBuffer_GetLineAt(uint32_t ptr, uint32_t id) {
  return (uint32_t )&(((PxRenderBuffer *)ptr)->getLines()[id]);
}
static uint32_t PxRenderBuffer_GetNbPoints(uint32_t ptr) {
  return ((PxRenderBuffer *)ptr)->getNbPoints();
}
static uint32_t PxRenderBuffer_GetPointAt(uint32_t ptr, uint32_t id) {
  return (uint32_t )&(((PxRenderBuffer *)ptr)->getPoints()[id]);
}
static uint32_t PxRenderBuffer_GetNbTriangles(uint32_t ptr) {
  return ((PxRenderBuffer *)ptr)->getNbTriangles();
}
static uint32_t PxRenderBuffer_GetTriangleAt(uint32_t ptr, uint32_t id) {
  return (uint32_t )&(((PxRenderBuffer *)ptr)->getTriangles()[id]);
}

// PxDebugLine
static uint32_t PxDebugLine_GetPos0(uint32_t ptr) {
  return (uint32_t)&(((PxDebugLine *)ptr)->pos0);
}
static uint32_t PxDebugLine_GetPos1(uint32_t ptr) {
  return (uint32_t)&(((PxDebugLine *)ptr)->pos1);
}
static uint32_t PxDebugLine_GetColor0(uint32_t ptr) {
  return ((PxDebugLine *)ptr)->color0;
}
static uint32_t PxDebugLine_GetColor1(uint32_t ptr) {
  return ((PxDebugLine *)ptr)->color1;
}

//----------------------------------------------------------------------------

EMSCRIPTEN_BINDINGS(physx) {

  constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);
  constant("LIB_VERSION", __LIB_VERSION__);

  // Global functions
  // These are generaly system/scene level initialization
  function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());
  function("PxInitExtensions", &PxInitExtensions, allow_raw_pointers());
  function("PxDefaultCpuDispatcherCreate", &PxDefaultCpuDispatcherCreate,
           allow_raw_pointers());
  function("PxCreateBasePhysics", &PxCreateBasePhysics, allow_raw_pointers());
  function("PxCreatePhysics", &PxCreatePhysics, allow_raw_pointers());
  function("PxRegisterArticulations", &PxRegisterArticulations,
           allow_raw_pointers());
  function("PxRegisterArticulationsReducedCoordinate",
           &PxRegisterArticulationsReducedCoordinate, allow_raw_pointers());
  function("PxRegisterHeightFields", &PxRegisterHeightFields,
           allow_raw_pointers());
  function("PxCreateCooking", &PxCreateCooking, allow_raw_pointers());
  function("PxCreatePlane", &PxCreatePlane, allow_raw_pointers());
  function("getDefaultSceneDesc", &getDefaultSceneDesc, allow_raw_pointers());
  function("getGContacts", &getGContacts, allow_raw_pointers());
  function("createCapsuleCharacterController",
           &createCapsuleCharacterController, allow_raw_pointers());
  function("createBoxCharacterController", &createBoxCharacterController,
           allow_raw_pointers());

  class_<PxSimulationEventCallback>("PxSimulationEventCallback")
      .allow_subclass<PxSimulationEventCallbackWrapper>(
          "PxSimulationEventCallbackWrapper");

  // Joints
  function("PxFixedJointCreate", &PxFixedJointCreate, allow_raw_pointers());
  function("PxRevoluteJointCreate", &PxRevoluteJointCreate,
           allow_raw_pointers());
  function("PxSphericalJointCreate", &PxSphericalJointCreate,
           allow_raw_pointers());
  function("PxDistanceJointCreate", &PxDistanceJointCreate,
           allow_raw_pointers());
  function("PxPrismaticJointCreate", &PxPrismaticJointCreate,
           allow_raw_pointers());
  function("PxD6JointCreate", &PxD6JointCreate, allow_raw_pointers());

  enum_<PxConstraintFlag::Enum>("PxConstraintFlag")
      .value("eBROKEN", PxConstraintFlag::Enum::eBROKEN)
      .value("eCOLLISION_ENABLED", PxConstraintFlag::Enum::eCOLLISION_ENABLED)
      .value("ePROJECTION", PxConstraintFlag::ePROJECTION);

  class_<PxSpring>("PxSpring")
      .property("stiffness", &PxSpring::stiffness)
      .property("damping", &PxSpring::damping);

  class_<PxJointLimitParameters>("PxJointLimitParameters")
      .property("restitution", &PxJointLimitParameters::restitution)
      .property("damping", &PxJointLimitParameters::damping)
      .property("stiffness", &PxJointLimitParameters::stiffness)
      .property("bounceThreshold", &PxJointLimitParameters::bounceThreshold)
      .property("contactDistance", &PxJointLimitParameters::contactDistance)
      .function("isValid", &PxJointLimitParameters::isValid)
      .function("isSoft", &PxJointLimitParameters::isSoft);

  class_<PxJointLimitCone, base<PxJointLimitParameters>>("PxJointLimitCone")
      .constructor<PxReal, PxReal>()
      .constructor<PxReal, PxReal, PxReal>()
      .property("yAngle", &PxJointLimitCone::yAngle)
      .property("zAngle", &PxJointLimitCone::zAngle);

  class_<PxJointLinearLimitPair, base<PxJointLimitParameters>>(
      "PxJointLinearLimitPair")
      .constructor<const PxTolerancesScale &, PxReal, PxReal>()
      .constructor<const PxTolerancesScale &, PxReal, PxReal, PxReal>()
      .property("upper", &PxJointLinearLimitPair::upper)
      .property("lower", &PxJointLinearLimitPair::lower);

  class_<PxJointAngularLimitPair, base<PxJointLimitParameters>>(
      "PxJointAngularLimitPair")
      .constructor<PxReal, PxReal>()
      .constructor<PxReal, PxReal, PxReal>()
      .property("upper", &PxJointAngularLimitPair::upper)
      .property("lower", &PxJointAngularLimitPair::lower);

  class_<PxJoint>("PxJoint")
      .function("setActors", &PxJoint::setActors, allow_raw_pointers())
      .function("setLocalPose", optional_override([](PxJoint &joint, PxU8 index,
                                                     PxTransform &pos) {
                  joint.setLocalPose(PxJointActorIndex::Enum(index), pos);
                }))
      .function("setBreakForce", &PxJoint::setBreakForce)
      .function("setConstraintFlag",
                optional_override([](PxJoint &joint, PxU16 flag, bool v) {
                  joint.setConstraintFlag(PxConstraintFlag::Enum(flag), v);
                }))
      .function("setConstraintFlags",
                optional_override([](PxJoint &joint, PxU16 flags) {
                  joint.setConstraintFlags(PxConstraintFlags(flags));
                }))
      .function("release", &PxJoint::release);
  class_<PxSphericalJoint, base<PxJoint>>("PxSphericalJoint");
  class_<PxRevoluteJoint, base<PxJoint>>("PxRevoluteJoint")
      .function("getAngle", &PxRevoluteJoint::getAngle)
      .function("getVelocity", &PxRevoluteJoint::getVelocity)
      .function("setLimit", &PxRevoluteJoint::setLimit)
      .function("getLimit", &PxRevoluteJoint::getLimit)
      .function("setDriveVelocity", &PxRevoluteJoint::setDriveVelocity)
      .function("getDriveVelocity", &PxRevoluteJoint::getDriveVelocity)
      .function("setDriveForceLimit", &PxRevoluteJoint::setDriveForceLimit)
      .function("getDriveForceLimit", &PxRevoluteJoint::getDriveForceLimit)
      .function("getDriveGearRatio", &PxRevoluteJoint::getDriveGearRatio)
      .function("setDriveGearRatio", &PxRevoluteJoint::setDriveGearRatio)
      .function(
          "setRevoluteJointFlag",
          optional_override([](PxRevoluteJoint &joint, PxU16 flag, bool v) {
            joint.setRevoluteJointFlag(PxRevoluteJointFlag::Enum(flag), v);
          }))
      .function("setRevoluteJointFlags",
                optional_override([](PxRevoluteJoint &joint, PxU16 flags) {
                  joint.setRevoluteJointFlags(PxRevoluteJointFlags(flags));
                }))
      .function("setProjectionLinearTolerance",
                &PxRevoluteJoint::setProjectionLinearTolerance)
      .function("getProjectionLinearTolerance",
                &PxRevoluteJoint::getProjectionLinearTolerance)
      .function("setProjectionAngularTolerance",
                &PxRevoluteJoint::setProjectionAngularTolerance)
      .function("getProjectionAngularTolerance",
                &PxRevoluteJoint::getProjectionAngularTolerance);
  class_<PxFixedJoint, base<PxJoint>>("PxFixedJoint")
      .function("setProjectionLinearTolerance",
                &PxFixedJoint::setProjectionLinearTolerance)
      .function("setProjectionAngularTolerance",
                &PxFixedJoint::setProjectionAngularTolerance);
  class_<PxDistanceJoint, base<PxJoint>>("PxDistanceJoint")
      .function("getDistance", &PxDistanceJoint::getDistance)
      .function("setMinDistance", &PxDistanceJoint::setMinDistance)
      .function("getMinDistance", &PxDistanceJoint::getMinDistance)
      .function("setMaxDistance", &PxDistanceJoint::setMaxDistance)
      .function("getMaxDistance", &PxDistanceJoint::getMaxDistance)
      .function("setTolerance", &PxDistanceJoint::setTolerance)
      .function("getTolerance", &PxDistanceJoint::getTolerance)
      .function("setStiffness", &PxDistanceJoint::setStiffness)
      .function("getStiffness", &PxDistanceJoint::getStiffness)
      .function("setDamping", &PxDistanceJoint::setDamping)
      .function("getDamping", &PxDistanceJoint::getDamping)
      .function("setDistanceJointFlags",
                optional_override([](PxDistanceJoint &joint, PxU16 flags) {
                  joint.setDistanceJointFlags(PxDistanceJointFlags(flags));
                }));
  class_<PxPrismaticJoint, base<PxJoint>>("PxPrismaticJoint");

  enum_<PxD6Axis::Enum>("PxD6Axis")
      .value("eX", PxD6Axis::Enum::eX)
      .value("eY", PxD6Axis::Enum::eY)
      .value("eZ", PxD6Axis::Enum::eZ)
      .value("eTWIST", PxD6Axis::Enum::eTWIST)
      .value("eSWING1", PxD6Axis::Enum::eSWING1)
      .value("eSWING2", PxD6Axis::Enum::eSWING2);

  enum_<PxD6Motion::Enum>("PxD6Motion")
      .value("eLOCKED", PxD6Motion::Enum::eLOCKED)
      .value("eLIMITED", PxD6Motion::Enum::eLIMITED)
      .value("eFREE", PxD6Motion::Enum::eFREE);

  class_<PxD6JointDrive, base<PxSpring>>("PxD6JointDrive")
      .constructor<>()
      .constructor<PxReal, PxReal, PxReal, bool>()
      .property("forceLimit", &PxD6JointDrive::forceLimit)
      .function("setAccelerationFlag",
                optional_override([](PxD6JointDrive &drive, bool enabled) {
                  if (enabled) {
                    drive.flags.set(PxD6JointDriveFlag::Enum::eACCELERATION);
                  } else {
                    drive.flags.clear(PxD6JointDriveFlag::Enum::eACCELERATION);
                  }
                }));

  enum_<PxD6Drive::Enum>("PxD6Drive")
      .value("eX", PxD6Drive::Enum::eX)
      .value("eY", PxD6Drive::Enum::eY)
      .value("eZ", PxD6Drive::Enum::eZ)
      .value("eSWING", PxD6Drive::Enum::eSWING)
      .value("eTWIST", PxD6Drive::Enum::eTWIST)
      .value("eSLERP", PxD6Drive::Enum::eSLERP);

  class_<PxD6Joint, base<PxJoint>>("PxD6Joint")
      .function("setMotion", &PxD6Joint::setMotion)
      .function("getMotion", &PxD6Joint::getMotion)
      .function(
          "setLinearLimit",
          select_overload<void(PxD6Axis::Enum, const PxJointLinearLimitPair &)>(
              &PxD6Joint::setLinearLimit))
      .function("setTwistLimit", &PxD6Joint::setTwistLimit)
      .function("setSwingLimit", &PxD6Joint::setSwingLimit)
      .function("setDrive", &PxD6Joint::setDrive)
      .function("setDrivePosition",
                select_overload<void(const PxTransform &, bool)>(
                    &PxD6Joint::setDrivePosition))
      .function("setDriveVelocity",
                select_overload<void(const PxVec3 &, const PxVec3 &, bool)>(
                    &PxD6Joint::setDriveVelocity));

  class_<PxAllocatorCallback>("PxAllocatorCallback");
  class_<PxDefaultAllocator, base<PxAllocatorCallback>>("PxDefaultAllocator")
      .constructor<>();

  class_<PxTolerancesScale>("PxTolerancesScale")
      .constructor<>()
      .property("speed", &PxTolerancesScale::speed)
      .property("length", &PxTolerancesScale::length);

  // Define PxVec3, PxQuat and PxTransform as value objects to allow sumerian
  // Vector3 and Quaternion to be used directly without the need to free the
  // memory
  value_object<PxVec3>("PxVec3")
      .field("x", &PxVec3::x)
      .field("y", &PxVec3::y)
      .field("z", &PxVec3::z);
  register_vector<PxVec3>("PxVec3Vector");
  value_object<PxQuat>("PxQuat")
      .field("x", &PxQuat::x)
      .field("y", &PxQuat::y)
      .field("z", &PxQuat::z)
      .field("w", &PxQuat::w);
  value_object<PxTransform>("PxTransform")
      .field("translation", &PxTransform::p)
      .field("rotation", &PxTransform::q);
  value_object<PxExtendedVec3>("PxExtendedVec3")
      .field("x", &PxExtendedVec3::x)
      .field("y", &PxExtendedVec3::y)
      .field("z", &PxExtendedVec3::z);
  value_object<PxBounds3>("PxBounds3")
      .field("minimum", &PxBounds3::minimum)
      .field("maximum", &PxBounds3::maximum);

  class_<PxContactPairPoint>("PxContactPairPoint")
      .property("normal", &PxContactPairPoint::normal)
      .property("impulse", &PxContactPairPoint::impulse)
      .property("position", &PxContactPairPoint::position)
      .property("separation", &PxContactPairPoint::separation);
  register_vector<PxContactPairPoint>("PxContactPairPointVector");

  enum_<PxIDENTITY>("PxIDENTITY").value("PxIdentity", PxIDENTITY::PxIdentity);

  enum_<PxVisualizationParameter::Enum>("PxVisualizationParameter")
      .value("eSCALE", PxVisualizationParameter::Enum::eSCALE)
      .value("eWORLD_AXES", PxVisualizationParameter::Enum::eWORLD_AXES)
      .value("eBODY_AXES", PxVisualizationParameter::Enum::eBODY_AXES)
      .value("eBODY_MASS_AXES", PxVisualizationParameter::Enum::eBODY_MASS_AXES)
      .value("eBODY_LIN_VELOCITY", PxVisualizationParameter::Enum::eBODY_LIN_VELOCITY)
      .value("eBODY_ANG_VELOCITY", PxVisualizationParameter::Enum::eBODY_ANG_VELOCITY)
      .value("eCONTACT_POINT", PxVisualizationParameter::Enum::eCONTACT_POINT)
      .value("eCONTACT_NORMAL", PxVisualizationParameter::Enum::eCONTACT_NORMAL)
      .value("eCONTACT_ERROR", PxVisualizationParameter::Enum::eCONTACT_ERROR)
      .value("eCONTACT_FORCE", PxVisualizationParameter::Enum::eCONTACT_FORCE)
      .value("eACTOR_AXES", PxVisualizationParameter::Enum::eACTOR_AXES)
      .value("eCOLLISION_AABBS", PxVisualizationParameter::Enum::eCOLLISION_AABBS)
      .value("eCOLLISION_SHAPES", PxVisualizationParameter::Enum::eCOLLISION_SHAPES)
      .value("eCOLLISION_AXES", PxVisualizationParameter::Enum::eCOLLISION_AXES)
      .value("eCOLLISION_COMPOUNDS", PxVisualizationParameter::Enum::eCOLLISION_COMPOUNDS)
      .value("eCOLLISION_FNORMALS", PxVisualizationParameter::Enum::eCOLLISION_FNORMALS)
      .value("eCOLLISION_EDGES", PxVisualizationParameter::Enum::eCOLLISION_EDGES)
      .value("eCOLLISION_STATIC", PxVisualizationParameter::Enum::eCOLLISION_STATIC)
      .value("eCOLLISION_DYNAMIC", PxVisualizationParameter::Enum::eCOLLISION_DYNAMIC)
      .value("eDEPRECATED_COLLISION_PAIRS", PxVisualizationParameter::Enum::eDEPRECATED_COLLISION_PAIRS)
      .value("eJOINT_LOCAL_FRAMES", PxVisualizationParameter::Enum::eJOINT_LOCAL_FRAMES)
      .value("eJOINT_LIMITS", PxVisualizationParameter::Enum::eJOINT_LIMITS)
      .value("eCULL_BOX", PxVisualizationParameter::Enum::eCULL_BOX)
      .value("eMBP_REGIONS", PxVisualizationParameter::Enum::eMBP_REGIONS)
      .value("eNUM_VALUES", PxVisualizationParameter::Enum::eNUM_VALUES);

  class_<PxDebugPoint>("PxDebugPoint")
      .constructor<PxVec3, PxU32>()
      .property("pos", &PxDebugPoint::pos)
      .property("color", &PxDebugPoint::color);

  class_<PxDebugLine>("PxDebugLine")
      .constructor<PxVec3, PxVec3, PxU32>()
      .property("pos0", &PxDebugLine::pos0)
      .property("pos1", &PxDebugLine::pos1)
      .property("color0", &PxDebugLine::color0)
      .property("color1", &PxDebugLine::color1);
  
  class_<PxDebugTriangle>("PxDebugTriangle")
      .constructor<PxVec3, PxVec3, PxVec3, PxU32>()
      .property("pos0", &PxDebugTriangle::pos0)
      .property("pos1", &PxDebugTriangle::pos1)
      .property("pos2", &PxDebugTriangle::pos2)
      .property("color0", &PxDebugTriangle::color0)
      .property("color1", &PxDebugTriangle::color1)
      .property("color2", &PxDebugTriangle::color2);

  // class_<PxDebugText>("PxDebugText")
  //     .constructor<>()
  //     .constructor<PxVec3, PxReal, PxU32, const char *>()
  //     .property("position", &PxDebugText::position)
  //     .property("size", &PxDebugText::size)
  //     .property("color", &PxDebugText::color)
  //     .property("string", &PxDebugText::string);

  function("PxRenderBuffer_GetNbLines", &PxRenderBuffer_GetNbLines);
  function("PxRenderBuffer_GetLineAt", &PxRenderBuffer_GetLineAt,
           allow_raw_pointers());
  function("PxRenderBuffer_GetNbPoints", &PxRenderBuffer_GetNbPoints);
  function("PxRenderBuffer_GetPointAt", &PxRenderBuffer_GetPointAt,
           allow_raw_pointers());
  function("PxRenderBuffer_GetNbTriangles", &PxRenderBuffer_GetNbTriangles);
  function("PxRenderBuffer_GetTriangleAt", &PxRenderBuffer_GetTriangleAt,
           allow_raw_pointers());

  function("PxDebugLine_GetPos0", &PxDebugLine_GetPos0);
  function("PxDebugLine_GetPos1", &PxDebugLine_GetPos1);
  function("PxDebugLine_GetColor0", &PxDebugLine_GetColor0);
  function("PxDebugLine_GetColor1", &PxDebugLine_GetColor1);

  class_<PxRenderBuffer>("PxRenderBuffer")
      .function("getNbLines", &PxRenderBuffer::getNbLines)
      //.function("getLines", &PxRenderBuffer::getLines, allow_raw_pointers())
      .function("getLineAt", optional_override( 
        [](const PxRenderBuffer* buffer, PxU32 index) {
            return &(buffer->getLines()[index]);
        }), allow_raw_pointers())
      .function("getNbPoints", &PxRenderBuffer::getNbPoints)
      //.function("getPoints", &PxRenderBuffer::getPoints, allow_raw_pointers())
      .function("getPointAt", optional_override( 
        [](const PxRenderBuffer* buffer, PxU32 index) {
            return &(buffer->getPoints()[index]);
        }), allow_raw_pointers())
      .function("getNbTriangles", &PxRenderBuffer::getNbTriangles)
      .function("getTriangleAt", optional_override( 
        [](const PxRenderBuffer* buffer, PxU32 index) {
            return &(buffer->getTriangles()[index]);
        }), allow_raw_pointers())
      //.function("getTriangles", &PxRenderBuffer::getTriangles,
      //          allow_raw_pointers())
      // .function("getNbTexts", &PxRenderBuffer::getNbTexts)
      // .function("getTexts", &PxRenderBuffer::getTexts, allow_raw_pointers())
      ;

  enum_<PxForceMode::Enum>("PxForceMode")
      .value("eFORCE", PxForceMode::Enum::eFORCE)
      .value("eIMPULSE", PxForceMode::Enum::eIMPULSE)
      .value("eVELOCITY_CHANGE", PxForceMode::Enum::eVELOCITY_CHANGE)
      .value("eACCELERATION", PxForceMode::Enum::eACCELERATION);

  class_<PxSceneDesc>("PxSceneDesc")
      .constructor<PxTolerancesScale>()
      .property("gravity", &PxSceneDesc::gravity);

  class_<PxFoundation>("PxFoundation")
      .function("release", &PxFoundation::release);

  class_<PxSceneFlags>("PxSceneFlags");
  enum_<PxSceneFlag::Enum>("PxSceneFlag")
      .value("eENABLE_ACTIVE_ACTORS ", PxSceneFlag::Enum::eENABLE_ACTIVE_ACTORS)
      .value("eENABLE_CCD", PxSceneFlag::Enum::eENABLE_CCD)
      .value("eDISABLE_CCD_RESWEEP", PxSceneFlag::Enum::eDISABLE_CCD_RESWEEP)
      .value("eADAPTIVE_FORCE", PxSceneFlag::Enum::eADAPTIVE_FORCE)
      .value("eENABLE_PCM", PxSceneFlag::Enum::eENABLE_PCM)
      .value("eDISABLE_CONTACT_REPORT_BUFFER_RESIZE",
             PxSceneFlag::Enum::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)
      .value("eDISABLE_CONTACT_CACHE",
             PxSceneFlag::Enum::eDISABLE_CONTACT_CACHE)
      .value("eREQUIRE_RW_LOCK", PxSceneFlag::Enum::eREQUIRE_RW_LOCK)
      .value("eENABLE_STABILIZATION", PxSceneFlag::Enum::eENABLE_STABILIZATION)
      .value("eENABLE_AVERAGE_POINT", PxSceneFlag::Enum::eENABLE_AVERAGE_POINT)
      .value("eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS",
             PxSceneFlag::Enum::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS)
      .value("eENABLE_ENHANCED_DETERMINISM",
             PxSceneFlag::Enum::eENABLE_ENHANCED_DETERMINISM)
      .value("eENABLE_FRICTION_EVERY_ITERATION",
             PxSceneFlag::Enum::eENABLE_FRICTION_EVERY_ITERATION);

  class_<PxScene>("PxScene")
      .function("release", &PxScene::release)
      .function("setGravity", &PxScene::setGravity)
      .function("getGravity", &PxScene::getGravity)
      .function("addActor", &PxScene::addActor, allow_raw_pointers())
      .function("removeActor", &PxScene::removeActor, allow_raw_pointers())
   #ifdef PX_WEB_BINDINGS_PVD
      .function("getScenePvdClient", &PxScene::getScenePvdClient,
                allow_raw_pointers())
   #endif
      .function("getActors", &PxScene::getActors, allow_raw_pointers())
      .function("setVisualizationCullingBox",
                &PxScene::setVisualizationCullingBox)
      .function("getVisualizationParameter", &PxScene::getVisualizationParameter)
      .function("setVisualizationParameter", &PxScene::setVisualizationParameter)
      // .function("getRenderBuffer", &PxScene::getRenderBuffer, allow_raw_pointers())
      .function("getRenderBuffer", optional_override( 
        [](PxScene &scene) {
            return &(scene.getRenderBuffer());
        }), allow_raw_pointers())
      .function("getRenderBufferPtr", optional_override( 
      [](PxScene &scene) {
          return uint32_t(&(scene.getRenderBuffer()));
      }), allow_raw_pointers())
      .function("simulate",
                optional_override([](PxScene &scene, PxReal elapsedTime,
                                     bool controlSimulation) {
                  gContactPointsNeedClear = true;
                  scene.simulate(elapsedTime, NULL, 0, 0, controlSimulation);
                  return;
                }))
      .function("fetchResults",
                optional_override([](PxScene &scene, bool block) {
                  // fetchResults uses an out pointer
                  // which embind can't represent
                  // so let's override.
                  bool fetched = scene.fetchResults(block);
                  return fetched;
                }))
      .function(
          "raycast",
          optional_override([](PxScene &scene, const PxVec3 &origin,
                               const PxVec3 &unitDir, const PxReal distance,
                               PxRaycastCallback &hitCall) {
            bool fetched = scene.raycast(origin, unitDir, distance, hitCall);
            return fetched;
          }))
      .function(
          "raycastSingle",
          optional_override([](PxScene &scene, const PxVec3 &origin,
                               const PxVec3 &unitDir, const PxReal distance,
                               PxU16 flags, PxRaycastHit &hit,
                               const PxSceneQueryFilterData &filterData,
                               PxSceneQueryFilterCallback *filterCall,
                               const PxSceneQueryCache *cache) {
            bool result = PxSceneQueryExt::raycastSingle(
                scene, origin, unitDir, distance, PxHitFlags(flags), hit,
                filterData, filterCall, cache);
            return result;
          }),
          allow_raw_pointers())
      .function("raycastAny",
                optional_override([](PxScene &scene, const PxVec3 &origin,
                                     const PxVec3 &unitDir,
                                     const PxReal distance, PxRaycastHit &hit,
                                     const PxSceneQueryFilterData &filterData,
                                     PxSceneQueryFilterCallback *filterCall,
                                     const PxSceneQueryCache *cache) {
                  return PxSceneQueryExt::raycastAny(scene, origin, unitDir,
                                                     distance, hit, filterData,
                                                     filterCall, cache);
                  ;
                }),
                allow_raw_pointers())
      .function("raycastMultiple",
                optional_override(
                    [](PxScene &scene, const PxVec3 &origin,
                       const PxVec3 &unitDir, const PxReal distance,
                       PxU16 flags, std::vector<PxRaycastHit> &hitBuffer,
                       PxU32 hbsize, const PxSceneQueryFilterData &filterData,
                       PxSceneQueryFilterCallback *filterCall,
                       const PxSceneQueryCache *cache) {
                      bool hitBlock = false;
                      return PxSceneQueryExt::raycastMultiple(
                          scene, origin, unitDir, distance, PxHitFlags(flags),
                          hitBuffer.data(), hbsize, hitBlock, filterData,
                          filterCall, cache);
                    }),
                allow_raw_pointers())
      .function("sweepSingle",
          optional_override([](PxScene &scene, 
                               const PxGeometry& geometry, const PxTransform& pose,
                               const PxVec3 &unitDir, const PxReal distance,
                               PxU16 flags, PxSweepHit &hit,
                               const PxSceneQueryFilterData &filterData,
                               PxSceneQueryFilterCallback *filterCall,
                               const PxSceneQueryCache *cache, PxReal inflation) {
            bool result = PxSceneQueryExt::sweepSingle(
                scene, geometry, pose, unitDir, distance, PxHitFlags(flags), hit,
                filterData, filterCall, cache, inflation);
            return result;
          }),
          allow_raw_pointers())
      .function("sweepMultiple",
                optional_override(
                    [](PxScene &scene,
                       const PxGeometry& geometry, const PxTransform& pose,
                       const PxVec3 &unitDir, const PxReal distance,
                       PxU16 flags, std::vector<PxSweepHit> &hitBuffer,
                       PxU32 hbsize, const PxSceneQueryFilterData &filterData,
                       PxSceneQueryFilterCallback *filterCall,
                       const PxSceneQueryCache *cache, PxReal inflation) {
                      bool hitBlock = false;
                      return PxSceneQueryExt::sweepMultiple(
                          scene, geometry, pose, unitDir, distance, PxHitFlags(flags),
                          hitBuffer.data(), hbsize, hitBlock, filterData,
                          filterCall, cache, inflation);
                    }),
                allow_raw_pointers())
      ;

  class_<PxQueryHit>("PxQueryHit")
      .function("getShape", optional_override([](PxQueryHit &block) {
                  return block.shape;
                }),
                allow_raw_pointers())
      .function("getActor", optional_override([](PxQueryHit &block) {
                  return block.actor;
                }),
                allow_raw_pointers());

  class_<PxLocationHit, base<PxQueryHit>>("PxLocationHit")
      .property("position", &PxLocationHit::position)
      .property("normal", &PxLocationHit::normal)
      .property("distance", &PxLocationHit::distance);
  class_<PxRaycastHit, base<PxLocationHit>>("PxRaycastHit").constructor<>();
  register_vector<PxRaycastHit>("PxRaycastHitVector");

  class_<PxRaycastCallback>("PxRaycastCallback")
      .property("block", &PxRaycastCallback::block)
      .property("hasBlock", &PxRaycastCallback::hasBlock)
      .allow_subclass<PxRaycastCallbackWrapper>(
          "PxRaycastCallbackWrapper", constructor<PxRaycastHit *, PxU32>());
  class_<PxRaycastBuffer, base<PxRaycastCallback>>("PxRaycastBuffer")
      .constructor<>();

  function("allocateRaycastHitBuffers", &allocateRaycastHitBuffers,
           allow_raw_pointers());

  class_<PxSweepHit, base<PxLocationHit>>("PxSweepHit").constructor<>();
  register_vector<PxSweepHit>("PxSweepHitVector");

  class_<PxSweepCallback>("PxSweepCallback")
      .property("block", &PxSweepCallback::block)
      .property("hasBlock", &PxSweepCallback::hasBlock)
      .allow_subclass<PxSweepCallbackWrapper>(
          "PxSweepCallbackWrapper", constructor<PxSweepHit *, PxU32>());
  class_<PxSweepBuffer, base<PxSweepCallback>>("PxSweepBuffer").constructor<>();

  function("allocateSweepHitBuffers", &allocateSweepHitBuffers,
           allow_raw_pointers());

  class_<PxHitFlags>("PxHitFlags").constructor<int>();
  enum_<PxHitFlag::Enum>("PxHitFlag")
      .value("eDEFAULT", PxHitFlag::Enum::eDEFAULT)
      .value("eMESH_BOTH_SIDES", PxHitFlag::Enum::eMESH_BOTH_SIDES)
      .value("eMESH_MULTIPLE", PxHitFlag::Enum::eMESH_MULTIPLE);

  class_<PxQueryFilterData>("PxQueryFilterData")
      .constructor<>()
      .function("setFlags",
                optional_override([](PxQueryFilterData &qf, const PxU16 f) {
                  qf.flags = PxQueryFlags(f);
                }))
      .function("setWords", optional_override([](PxQueryFilterData &qf,
                                                 const PxU32 f, const PxU16 i) {
                  if (i == 0)
                    qf.data.word0 = f;
                  else if (i == 1)
                    qf.data.word1 = f;
                  else if (i == 2)
                    qf.data.word2 = f;
                  else if (i == 3)
                    qf.data.word3 = f;
                }))
      .property("data", &PxQueryFilterData::data);
  class_<PxQueryFlags>("PxQueryFlags").constructor<int>();
  enum_<PxQueryFlag::Enum>("PxQueryFlag")
      .value("eANY_HIT", PxQueryFlag::Enum::eANY_HIT)
      .value("eDYNAMIC", PxQueryFlag::Enum::eDYNAMIC)
      .value("eSTATIC", PxQueryFlag::Enum::eSTATIC)
      .value("ePREFILTER", PxQueryFlag::Enum::ePREFILTER)
      .value("ePOSTFILTER", PxQueryFlag::Enum::ePOSTFILTER)
      .value("eNO_BLOCK", PxQueryFlag::Enum::eNO_BLOCK);
  enum_<PxQueryHitType::Enum>("PxQueryHitType")
      .value("eNONE", PxQueryHitType::Enum::eNONE)
      .value("eBLOCK", PxQueryHitType::Enum::eBLOCK)
      .value("eTOUCH", PxQueryHitType::Enum::eTOUCH);

  class_<PxQueryFilterCallback>("PxQueryFilterCallback")
      .allow_subclass<PxQueryFilterCallbackWrapper>(
          "PxQueryFilterCallbackWrapper", constructor<>());
  class_<PxQueryCache>("PxQueryCache");

  enum_<PxCombineMode::Enum>("PxCombineMode")
      .value("eAVERAGE", PxCombineMode::Enum::eAVERAGE)
      .value("eMIN", PxCombineMode::Enum::eMIN)
      .value("eMULTIPLY", PxCombineMode::Enum::eMULTIPLY)
      .value("eMAX", PxCombineMode::Enum::eMAX)
      .value("eN_VALUES", PxCombineMode::Enum::eN_VALUES)
      .value("ePAD_32", PxCombineMode::Enum::ePAD_32);

  class_<PxMaterial>("PxMaterial")
      .function("setDynamicFriction", &PxMaterial::setDynamicFriction)
      .function("setStaticFriction", &PxMaterial::setStaticFriction)
      .function("setRestitution", &PxMaterial::setRestitution)
      .function("getDynamicFriction", &PxMaterial::getDynamicFriction)
      .function("setFrictionCombineMode", &PxMaterial::setFrictionCombineMode)
      .function("setRestitutionCombineMode",
                &PxMaterial::setRestitutionCombineMode)
      .function("release", &PxMaterial::release);
  register_vector<PxMaterial *>("PxMaterialVector");
  // setMaterials has 'PxMaterial**' as an input, which is not representable
  // with embind This is overrided to use std::vector<PxMaterial*>
  class_<PxShape>("PxShape")
      .function("release", &PxShape::release)
      .function("getReferenceCount", &PxShape::getReferenceCount)
      .function("getFlags", &PxShape::getFlags)
      .function("setFlag", &PxShape::setFlag)
      .function("setLocalPose", &PxShape::setLocalPose)
      .function("setGeometry", &PxShape::setGeometry)
      .function("getBoxGeometry", &PxShape::getBoxGeometry,
                allow_raw_pointers())
      .function("getSphereGeometry", &PxShape::getSphereGeometry,
                allow_raw_pointers())
      .function("getPlaneGeometry", &PxShape::getPlaneGeometry,
                allow_raw_pointers())
      .function("setSimulationFilterData", &PxShape::setSimulationFilterData,
                allow_raw_pointers())
      .function("setSimulationFilterData", &PxShape::getSimulationFilterData,
                allow_raw_pointers())
      .function("setQueryFilterData", &PxShape::setQueryFilterData)
      .function("getQueryFilterData", &PxShape::getQueryFilterData,
                allow_raw_pointers())
      .function("setMaterials",
                optional_override([](PxShape &shape,
                                     std::vector<PxMaterial *> materials) {
                  return shape.setMaterials(materials.data(), materials.size());
                }))
      .function(
          "getWorldBounds",
          optional_override([](PxShape &shape, PxRigidActor &actor, float i) {
            return PxShapeExt::getWorldBounds(shape, actor, i);
          }));

  class_<PxPhysics>("PxPhysics")
      .function("release", &PxPhysics::release)
      .function("getTolerancesScale", &PxPhysics::getTolerancesScale)
      .function("createScene", &PxPhysics::createScene, allow_raw_pointers())
      .function(
          "createShape",
          select_overload<PxShape *(const PxGeometry &, const PxMaterial &,
                                    bool, PxShapeFlags)>(
              &PxPhysics::createShape),
          allow_raw_pointers())
      .function("createMaterial", &PxPhysics::createMaterial,
                allow_raw_pointers())
      .function("createRigidDynamic", &PxPhysics::createRigidDynamic,
                allow_raw_pointers())
      .function("createRigidStatic", &PxPhysics::createRigidStatic,
                allow_raw_pointers());

  class_<PxShapeFlags>("PxShapeFlags")
      .constructor<int>()
      .function("isSet", &PxShapeFlags::isSet);
  enum_<PxShapeFlag::Enum>("PxShapeFlag")
      .value("eSIMULATION_SHAPE", PxShapeFlag::Enum::eSIMULATION_SHAPE)
      .value("eSCENE_QUERY_SHAPE", PxShapeFlag::Enum::eSCENE_QUERY_SHAPE)
      .value("eTRIGGER_SHAPE", PxShapeFlag::Enum::eTRIGGER_SHAPE)
      .value("eVISUALIZATION", PxShapeFlag::Enum::eVISUALIZATION);

  enum_<PxActorFlag::Enum>("PxActorFlag")
      .value("eVISUALIZATION", PxActorFlag::Enum::eVISUALIZATION)
      .value("eDISABLE_GRAVITY", PxActorFlag::Enum::eDISABLE_GRAVITY)
      .value("eSEND_SLEEP_NOTIFIES", PxActorFlag::Enum::eSEND_SLEEP_NOTIFIES)
      .value("eDISABLE_SIMULATION", PxActorFlag::Enum::eDISABLE_SIMULATION);

  class_<PxErrorCallback>("PxErrorCallback");
  class_<PxDefaultErrorCallback, base<PxErrorCallback>>(
      "PxDefaultErrorCallback")
      .constructor<>();

  class_<PxBitAndByte>("PxBitAndByte")
      .function("isBitSet", &PxBitAndByte::isBitSet)
      .function("setBit", &PxBitAndByte::setBit)
      .function("clearBit", &PxBitAndByte::clearBit);

  class_<PxHeightFieldSample>("PxHeightFieldSample")
      .constructor()
      .property("height", &PxHeightFieldSample::height)
      .property("materialIndex0", &PxHeightFieldSample::materialIndex0)
      .property("materialIndex1", &PxHeightFieldSample::materialIndex1);
  register_vector<PxHeightFieldSample>("PxHeightFieldSampleVector");

  register_vector<PxU16>("PxU16Vector");
  class_<PxCooking>("PxCooking")
      .function("createConvexMesh",
                optional_override([](PxCooking &cooking,
                                     std::vector<PxVec3> &vertices,
                                     PxPhysics &physics) {
                  return createConvexMesh(vertices, cooking, physics);
                }),
                allow_raw_pointers())
      .function("createConvexMeshFromBuffer",
                optional_override([](PxCooking &cooking, int vertices,
                                     PxU32 vertCount, PxPhysics &physics) {
                  return createConvexMeshFromBuffer(vertices, vertCount,
                                                    cooking, physics);
                }),
                allow_raw_pointers())
      .function(
          "createTriMesh",
          optional_override([](PxCooking &cooking, int vertices,
                               PxU32 vertCount, int indices, PxU32 indexCount,
                               bool isU16, PxPhysics &physics) {
            return createTriMesh(vertices, vertCount, indices, indexCount,
                                 isU16, cooking, physics);
          }),
          allow_raw_pointers())
      .function("createTriMeshExt",
                optional_override(
                    [](PxCooking &cooking, std::vector<PxVec3> &vertices,
                       std::vector<PxU16> &indices, PxPhysics &physics) {
                      return createTriMeshExt(vertices, indices, cooking,
                                              physics);
                    }),
                allow_raw_pointers())
      .function(
          "createHeightFieldExt",
          optional_override([](PxCooking &cooking, PxU32 numCols, PxU32 numRows,
                               std::vector<PxHeightFieldSample> &samples,
                               PxPhysics &physics) {
            return createHeightFieldExt(numCols, numRows, samples, cooking,
                                        physics);
          }),
          allow_raw_pointers());
  class_<PxCookingParams>("PxCookingParams").constructor<PxTolerancesScale>();
  class_<PxCpuDispatcher>("PxCpuDispatcher");
  class_<PxBVHStructure>("PxBVHStructure");
  class_<PxBaseTask>("PxBaseTask");
  class_<PxDefaultCpuDispatcher, base<PxCpuDispatcher>>(
      "PxDefaultCpuDispatcher");

  // class_<PxFilterData>("PxFilterData")
  //     .constructor<PxU32, PxU32, PxU32, PxU32>()
  //     .property("word0", &PxFilterData::word0)
  //     .property("word1", &PxFilterData::word1)
  //     .property("word2", &PxFilterData::word2)
  //     .property("word3", &PxFilterData::word3);
  value_object<PxFilterData>("PxFilterData")
      .field("word0", &PxFilterData::word0)
      .field("word1", &PxFilterData::word1)
      .field("word2", &PxFilterData::word2)
      .field("word3", &PxFilterData::word3);

  class_<PxPairFlags>("PxPairFlags");
  class_<PxFilterFlags>("PxFilterFlags");

  enum_<PxPairFlag::Enum>("PxPairFlag");
  enum_<PxFilterFlag::Enum>("PxFilterFlag");

  class_<PxActor>("PxActor")
      .function("setActorFlag", &PxActor::setActorFlag)
      .function("release", &PxActor::release);

  class_<PxRigidActor, base<PxActor>>("PxRigidActor")
      .function("attachShape", &PxRigidActor::attachShape)
      .function("detachShape", &PxRigidActor::detachShape)
      .function("getGlobalPose", &PxRigidActor::getGlobalPose,
                allow_raw_pointers())
      .function("setGlobalPose", &PxRigidActor::setGlobalPose,
                allow_raw_pointers());

  class_<PxRigidBody, base<PxRigidActor>>("PxRigidBody")
      .function("setAngularDamping", &PxRigidBody::setAngularDamping)
      .function("getAngularDamping", &PxRigidBody::getAngularDamping)
      .function("setLinearDamping", &PxRigidBody::setLinearDamping)
      .function("getLinearDamping", &PxRigidBody::getLinearDamping)
      .function("setAngularVelocity", &PxRigidBody::setAngularVelocity)
      .function("getAngularVelocity", &PxRigidBody::getAngularVelocity)
      .function("setMass", &PxRigidBody::setMass)
      .function("getMass", &PxRigidBody::getMass)
      .function("setCMassLocalPose", &PxRigidBody::setCMassLocalPose,
                allow_raw_pointers())
      .function("setLinearVelocity", &PxRigidBody::setLinearVelocity)
      .function("getLinearVelocity", &PxRigidBody::getLinearVelocity)
      .function("clearForce", &PxRigidBody::clearForce)
      .function("clearTorque", &PxRigidBody::clearTorque)
      // .function("addForceAtPos", optional_override(
      //                                [](PxRigidBody &body, const PxVec3
      //                                &force, const PxVec3 &pos) {
      //                                  PxRigidBodyExt::addForceAtPos(body,
      //                                  force, pos, PxForceMode::eFORCE,
      //                                  true);
      //                                }))
      // .function("addForceAtLocalPos", optional_override(
      //                                     [](PxRigidBody &body, const PxVec3
      //                                     &force, const PxVec3 &pos) {
      //                                       PxRigidBodyExt::addForceAtLocalPos(body,
      //                                       force, pos, PxForceMode::eFORCE,
      //                                       true);
      //                                     }))
      // .function("addLocalForceAtLocalPos", optional_override(
      //                                          [](PxRigidBody &body, const
      //                                          PxVec3 &force, const PxVec3
      //                                          &pos) {
      //                                            PxRigidBodyExt::addLocalForceAtLocalPos(body,
      //                                            force, pos,
      //                                            PxForceMode::eFORCE, true);
      //                                          }))
      // .function("addImpulseAtPos", optional_override(
      //                                  [](PxRigidBody &body, const PxVec3
      //                                  &impulse, const PxVec3 &pos) {
      //                                    PxRigidBodyExt::addForceAtPos(body,
      //                                    impulse, pos, PxForceMode::eIMPULSE,
      //                                    true);
      //                                  }))
      // .function("addImpulseAtLocalPos", optional_override(
      //                                       [](PxRigidBody &body, const
      //                                       PxVec3 &impulse, const PxVec3
      //                                       &pos) {
      //                                         PxRigidBodyExt::addForceAtLocalPos(body,
      //                                         impulse, pos,
      //                                         PxForceMode::eIMPULSE, true);
      //                                       }))
      // .function("addLocalImpulseAtLocalPos", optional_override(
      //                                            [](PxRigidBody &body, const
      //                                            PxVec3 &impulse, const
      //                                            PxVec3 &pos) {
      //                                              PxRigidBodyExt::addLocalForceAtLocalPos(body,
      //                                              impulse, pos,
      //                                              PxForceMode::eIMPULSE,
      //                                              true);
      //                                            }))
      .function("applyImpulse",
                optional_override([](PxRigidBody &body, const PxVec3 &impulse,
                                     const PxVec3 &pos) {
                  if (!impulse.isZero()) {
                    const PxVec3 torque = pos.cross(impulse);
                    body.addForce(impulse, PxForceMode::eIMPULSE, true);
                    if (!torque.isZero())
                      body.addTorque(torque, PxForceMode::eIMPULSE, true);
                  }
                }))
      .function("applyLocalImpulse",
                optional_override([](PxRigidBody &body, const PxVec3 &impulse,
                                     const PxVec3 &pos) {
                  if (!impulse.isZero()) {
                    // transform vector to world frame
                    const PxTransform bodyPose = body.getGlobalPose();
                    const PxVec3 worldImpulse = bodyPose.rotate(impulse);
                    const PxVec3 worldPos = bodyPose.rotate(pos);
                    body.addForce(worldImpulse, PxForceMode::eIMPULSE, true);
                    const PxVec3 torque = worldPos.cross(worldImpulse);
                    if (!torque.isZero())
                      body.addTorque(torque, PxForceMode::eIMPULSE, true);
                  }
                }))
      .function("applyForce",
                optional_override([](PxRigidBody &body, const PxVec3 &force,
                                     const PxVec3 &pos) {
                  if (!force.isZero()) {
                    body.addForce(force, PxForceMode::eFORCE, true);
                    const PxVec3 torque = pos.cross(force);
                    if (!torque.isZero())
                      body.addTorque(torque, PxForceMode::eFORCE, true);
                  }
                }))
      .function("applyLocalForce",
                optional_override([](PxRigidBody &body, const PxVec3 &force,
                                     const PxVec3 &pos) {
                  if (!force.isZero()) {
                    // transform vector to world frame
                    const PxTransform bodyPose = body.getGlobalPose();
                    const PxVec3 worldForce = bodyPose.rotate(force);
                    const PxVec3 worldPos = bodyPose.rotate(pos);
                    body.addForce(worldForce, PxForceMode::eFORCE, true);
                    const PxVec3 torque = worldPos.cross(worldForce);
                    if (!torque.isZero())
                      body.addTorque(torque, PxForceMode::eFORCE, true);
                  }
                }))
      .function("addTorque",
                optional_override([](PxRigidBody &body, const PxVec3 &torque) {
                  body.addTorque(torque, PxForceMode::eFORCE, true);
                }))
      .function("setRigidBodyFlag", &PxRigidBody::setRigidBodyFlag)
      .function("getRigidBodyFlags", optional_override([](PxRigidBody &body) {
                  return (bool)(body.getRigidBodyFlags() &
                                PxRigidBodyFlag::eKINEMATIC);
                }))
      .function("setMassAndUpdateInertia",
                optional_override([](PxRigidBody &body, PxReal mass) {
                  return PxRigidBodyExt::setMassAndUpdateInertia(body, mass,
                                                                 NULL, false);
                }))
      .function("setMassSpaceInertiaTensor",
                &PxRigidBody::setMassSpaceInertiaTensor);

  class_<PxRigidBodyFlags>("PxRigidBodyFlags");
  enum_<PxRigidBodyFlag::Enum>("PxRigidBodyFlag")
      .value("eKINEMATIC", PxRigidBodyFlag::Enum::eKINEMATIC)
      .value("eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES",
             PxRigidBodyFlag::Enum::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES)
      .value("eENABLE_CCD", PxRigidBodyFlag::Enum::eENABLE_CCD)
      .value("eENABLE_CCD_FRICTION",
             PxRigidBodyFlag::Enum::eENABLE_CCD_FRICTION)
      .value("eENABLE_POSE_INTEGRATION_PREVIEW",
             PxRigidBodyFlag::Enum::eENABLE_POSE_INTEGRATION_PREVIEW)
      .value("eENABLE_SPECULATIVE_CCD",
             PxRigidBodyFlag::Enum::eENABLE_SPECULATIVE_CCD)
      .value("eENABLE_CCD_MAX_CONTACT_IMPULSE",
             PxRigidBodyFlag::Enum::eENABLE_CCD_MAX_CONTACT_IMPULSE)
      .value("eRETAIN_ACCELERATIONS",
             PxRigidBodyFlag::Enum::eRETAIN_ACCELERATIONS);

  class_<PxRigidStatic, base<PxRigidActor>>("PxRigidStatic");
  class_<PxRigidDynamic, base<PxRigidBody>>("PxRigidDynamic")
      .function("wakeUp", &PxRigidDynamic::wakeUp)
      .function("putToSleep", &PxRigidDynamic::putToSleep)
      .function("isSleeping", &PxRigidDynamic::isSleeping)
      .function("setWakeCounter", &PxRigidDynamic::setWakeCounter)
      .function("getWakeCounter", &PxRigidDynamic::getWakeCounter)
      .function("setSleepThreshold", &PxRigidDynamic::setSleepThreshold)
      .function("getSleepThreshold", &PxRigidDynamic::getSleepThreshold)
      .function("setKinematicTarget", &PxRigidDynamic::setKinematicTarget)
      .function("setRigidDynamicLockFlag",
                &PxRigidDynamic::setRigidDynamicLockFlag)
      .function("setRigidDynamicLockFlags",
                &PxRigidDynamic::setRigidDynamicLockFlags);
  class_<PxRigidDynamicLockFlags>("PxRigidDynamicLockFlags").constructor<int>();
  enum_<PxRigidDynamicLockFlag::Enum>("PxRigidDynamicLockFlag")
      .value("eLOCK_LINEAR_X", PxRigidDynamicLockFlag::Enum::eLOCK_LINEAR_X)
      .value("eLOCK_LINEAR_Y", PxRigidDynamicLockFlag::Enum::eLOCK_LINEAR_Y)
      .value("eLOCK_LINEAR_Z", PxRigidDynamicLockFlag::Enum::eLOCK_LINEAR_Z)
      .value("eLOCK_ANGULAR_X", PxRigidDynamicLockFlag::Enum::eLOCK_ANGULAR_X)
      .value("eLOCK_ANGULAR_Y", PxRigidDynamicLockFlag::Enum::eLOCK_ANGULAR_Y)
      .value("eLOCK_ANGULAR_Z", PxRigidDynamicLockFlag::Enum::eLOCK_ANGULAR_Z);

  /** Geometry **/
  class_<PxGeometry>("PxGeometry");
  class_<PxBoxGeometry, base<PxGeometry>>("PxBoxGeometry")
      .constructor<PxVec3>()
      .function("setHalfExtents",
                optional_override([](PxBoxGeometry &geo, PxVec3 hf) {
                  geo.halfExtents = hf;
                }));

  class_<PxSphereGeometry, base<PxGeometry>>("PxSphereGeometry")
      .constructor<float>()
      .function("isValid", &PxSphereGeometry::isValid)
      .function("setRadius",
                optional_override(
                    [](PxSphereGeometry &geo, PxReal r) { geo.radius = r; }));

  class_<PxCapsuleGeometry, base<PxGeometry>>("PxCapsuleGeometry")
      .constructor<float, float>()
      .function("isValid", &PxCapsuleGeometry::isValid)
      .function("setRadius",
                optional_override(
                    [](PxCapsuleGeometry &geo, PxReal r) { geo.radius = r; }))
      .function("setHalfHeight",
                optional_override([](PxCapsuleGeometry &geo, PxReal hf) {
                  geo.halfHeight = hf;
                }));

  class_<PxTriangleMesh>("PxTriangleMesh")
      .function("release", &PxTriangleMesh::release);

  class_<PxTriangleMeshGeometry, base<PxGeometry>>("PxTriangleMeshGeometry")
      .constructor<PxTriangleMesh *, const PxMeshScale &, PxMeshGeometryFlags>()
      .function("setScale", optional_override([](PxTriangleMeshGeometry &geo,
                                                 PxMeshScale &scale) {
                  geo.scale.scale = scale.scale;
                  geo.scale.rotation = scale.rotation;
                }))
      .function("isValid", &PxTriangleMeshGeometry::isValid);

  class_<PxMeshGeometryFlags>("PxMeshGeometryFlags").constructor<int>();
  enum_<PxMeshGeometryFlag::Enum>("PxMeshGeometryFlag")
      .value("eDOUBLE_SIDED", PxMeshGeometryFlag::Enum::eDOUBLE_SIDED);

  class_<PxPlaneGeometry, base<PxGeometry>>("PxPlaneGeometry")
      .constructor<>()
      .function("isValid", &PxPlaneGeometry::isValid);

  class_<PxConvexMesh>("PxConvexMesh")
      .function("release", &PxConvexMesh::release);
  class_<PxConvexMeshGeometry, base<PxGeometry>>("PxConvexMeshGeometry")
      .constructor<PxConvexMesh *, const PxMeshScale &,
                   PxConvexMeshGeometryFlags>()
      .function("setScale", optional_override([](PxConvexMeshGeometry &geo,
                                                 PxMeshScale &scale) {
                  geo.scale.scale = scale.scale;
                  geo.scale.rotation = scale.rotation;
                }))
      .function("isValid", &PxConvexMeshGeometry::isValid);

  class_<PxMeshScale>("PxMeshScale")
      .constructor<const PxVec3 &, const PxQuat &>()
      .function("setScale",
                optional_override(
                    [](PxMeshScale &ms, PxVec3 &scale) { ms.scale = scale; }))
      .function("setRotation",
                optional_override(
                    [](PxMeshScale &ms, PxQuat &rot) { ms.rotation = rot; }));

  class_<PxConvexMeshGeometryFlags>("PxConvexMeshGeometryFlags")
      .constructor<int>();
  enum_<PxConvexMeshGeometryFlag::Enum>("PxConvexMeshGeometryFlag")
      .value("eTIGHT_BOUNDS", PxConvexMeshGeometryFlag::Enum::eTIGHT_BOUNDS);

  class_<PxHeightField>("PxHeightField")
      .function("release", &PxHeightField::release);
  class_<PxHeightFieldGeometry, base<PxGeometry>>("PxHeightFieldGeometry")
      .constructor<PxHeightField *, PxMeshGeometryFlags, PxReal, PxReal,
                   PxReal>()
      .function("isValid", &PxHeightFieldGeometry::isValid);

  /** End Geometry **/

  class_<PxPlane>("PxPlane").constructor<float, float, float, float>();

  /** Character Controller **/

  function("PxCreateControllerManager", &PxCreateControllerManager,
           allow_raw_pointers());

  enum_<PxControllerShapeType::Enum>("PxControllerShapeType")
      .value("eBOX", PxControllerShapeType::Enum::eBOX)
      .value("eCAPSULE", PxControllerShapeType::Enum::eCAPSULE)
      .value("eFORCE_DWORD", PxControllerShapeType::Enum::eFORCE_DWORD);

  enum_<PxCapsuleClimbingMode::Enum>("PxCapsuleClimbingMode")
      .value("eEASY", PxCapsuleClimbingMode::Enum::eEASY)
      .value("eCONSTRAINED", PxCapsuleClimbingMode::Enum::eCONSTRAINED)
      .value("eLAST", PxCapsuleClimbingMode::Enum::eLAST);

  enum_<PxControllerNonWalkableMode::Enum>("PxControllerNonWalkableMode")
      .value("ePREVENT_CLIMBING",
             PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING)
      .value("ePREVENT_CLIMBING_AND_FORCE_SLIDING",
             PxControllerNonWalkableMode::Enum::
                 ePREVENT_CLIMBING_AND_FORCE_SLIDING);

  class_<PxControllerManager>("PxControllerManager")
      .function("createController", &PxControllerManager::createController,
                allow_raw_pointers())
      .function("setTessellation", &PxControllerManager::setTessellation)
      .function("setOverlapRecoveryModule",
                &PxControllerManager::setOverlapRecoveryModule)
      .function("setPreciseSweeps", &PxControllerManager::setPreciseSweeps)
      .function("setPreventVerticalSlidingAgainstCeiling",
                &PxControllerManager::setPreventVerticalSlidingAgainstCeiling)
      .function("shiftOrigin", &PxControllerManager::shiftOrigin);

  class_<PxController>("PxController")
      .function("release", &PxController::release)
      // .function("move", &PxController::move, allow_raw_pointers())
      .function("move",
                optional_override([](PxController &controller,
                                     const PxVec3 &disp, PxF32 minDist,
                                     PxF32 elapsedTime, PxFilterData filterData,
                                     PxQueryFilterCallback *cb) -> uint32_t {
                  PxControllerFilters controllerFilters(&filterData, cb);
                  return controller.move(disp, minDist, elapsedTime,
                                         controllerFilters);
                }),
                allow_raw_pointers())
      .function("setPosition", &PxController::setPosition)
      .function("getPosition", &PxController::getPosition)
      .function("setStepOffset", &PxController::setStepOffset)
      .function("getStepOffset", &PxController::getStepOffset)
      .function("setContactOffset", &PxController::setContactOffset)
      .function("getContactOffset", &PxController::getContactOffset)
      .function("setSlopeLimit", &PxController::setSlopeLimit)
      .function("getSlopeLimit", &PxController::getSlopeLimit)
      .function("setCollision",
                optional_override([](PxController &ctrl, bool enable) {
                  PxRigidDynamic *actor = ctrl.getActor();
                  PxShape *shape;
                  actor->getShapes(&shape, 1);
                  shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, enable);
                  return;
                }))
      .function("setQuery",
                optional_override([](PxController &ctrl, bool enable) {
                  PxRigidDynamic *actor = ctrl.getActor();
                  PxShape *shape;
                  actor->getShapes(&shape, 1);
                  shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, enable);
                  return;
                }))
      .function("setSimulationFilterData",
                optional_override([](PxController &ctrl, PxFilterData &data) {
                  PxRigidDynamic *actor = ctrl.getActor();
                  PxShape *shape;
                  actor->getShapes(&shape, 1);
                  shape->setSimulationFilterData(data);
                  return;
                }))
      .function("setQueryFilterData",
                optional_override([](PxController &ctrl, PxFilterData &data) {
                  PxRigidDynamic *actor = ctrl.getActor();
                  PxShape *shape;
                  actor->getShapes(&shape, 1);
                  shape->setQueryFilterData(data);
                  return;
                }))
      .function("getShape",
          optional_override( [](PxController &ctrl) { 
                      PxRigidDynamic *actor = ctrl.getActor();
                      PxShape *shape;
                      actor->getShapes(&shape, 1);
                      return shape;
                     }), allow_raw_pointers());

  class_<PxCapsuleController, base<PxController>>("PxCapsuleController")
      .function("getRadius", &PxCapsuleController::getRadius)
      .function("setRadius", &PxCapsuleController::setRadius)
      .function("getHeight", &PxCapsuleController::getHeight)
      .function("setHeight", &PxCapsuleController::setHeight)
      .function("getClimbingMode", &PxCapsuleController::getClimbingMode)
      .function("setClimbingMode", &PxCapsuleController::setClimbingMode);

  class_<PxBoxController, base<PxController>>("PxBoxController")
      .function("getHalfHeight", &PxBoxController::getHalfHeight)
      .function("getHalfSideExtent", &PxBoxController::getHalfSideExtent)
      .function("getHalfForwardExtent", &PxBoxController::getHalfForwardExtent)
      .function("setHalfHeight", &PxBoxController::setHalfHeight)
      .function("setHalfSideExtent", &PxBoxController::setHalfSideExtent)
      .function("setHalfForwardExtent", &PxBoxController::setHalfForwardExtent);

  class_<PxControllerDesc>("PxControllerDesc")
      .function("isValid", &PxControllerDesc::isValid)
      .function("getType", &PxControllerDesc::getType)
      .property("position", &PxControllerDesc::position)
      .property("upDirection", &PxControllerDesc::upDirection)
      .property("slopeLimit", &PxControllerDesc::slopeLimit)
      .property("invisibleWallHeight", &PxControllerDesc::invisibleWallHeight)
      .property("maxJumpHeight", &PxControllerDesc::maxJumpHeight)
      .property("contactOffset", &PxControllerDesc::contactOffset)
      .property("stepOffset", &PxControllerDesc::stepOffset)
      .property("density", &PxControllerDesc::density)
      .property("scaleCoeff", &PxControllerDesc::scaleCoeff)
      .property("volumeGrowth", &PxControllerDesc::volumeGrowth)
      .property("nonWalkableMode", &PxControllerDesc::nonWalkableMode)
      // `material` property doesn't work as-is so we create a setMaterial
      .function(
          "setMaterial",
          optional_override([](PxControllerDesc &desc, PxMaterial *material) {
            return desc.material = material;
          }),
          allow_raw_pointers())
      .function(
          "setReportCallback",
          optional_override([](PxControllerDesc &desc,
                               PxUserControllerHitReport *reportCallback) {
            return desc.reportCallback = reportCallback;
          }),
          allow_raw_pointers());

  class_<PxCapsuleControllerDesc, base<PxControllerDesc>>(
      "PxCapsuleControllerDesc")
      .constructor<>()
      .function("isValid", &PxCapsuleControllerDesc::isValid)
      .property("radius", &PxCapsuleControllerDesc::radius)
      .property("height", &PxCapsuleControllerDesc::height)
      .property("climbingMode", &PxCapsuleControllerDesc::climbingMode);

  class_<PxBoxControllerDesc, base<PxControllerDesc>>("PxBoxControllerDesc")
      .constructor<>()
      .function("isValid", &PxBoxControllerDesc::isValid)
      .property("halfHeight", &PxBoxControllerDesc::halfHeight)
      .property("halfSideExtent", &PxBoxControllerDesc::halfSideExtent)
      .property("halfForwardExtent", &PxBoxControllerDesc::halfForwardExtent);

  class_<PxObstacleContext>("PxObstacleContext");

  class_<PxControllerFilters>("PxControllerFilters")
      .constructor<const PxFilterData *, PxQueryFilterCallback *,
                   PxControllerFilterCallback *>()
      .property("mFilterFlags", &PxControllerFilters::mFilterFlags);

  class_<PxControllerFilterCallback>("ControllerFilterCallback");

  class_<PxControllerCollisionFlags>("ControllerCollisionFlags")
      .constructor<PxU32>()
      .function("isSet", &PxControllerCollisionFlags::isSet);

  enum_<PxControllerCollisionFlag::Enum>("PxControllerCollisionFlag")
      .value("eCOLLISION_SIDES",
             PxControllerCollisionFlag::Enum::eCOLLISION_SIDES)
      .value("eCOLLISION_UP", PxControllerCollisionFlag::Enum::eCOLLISION_UP)
      .value("eCOLLISION_DOWN",
             PxControllerCollisionFlag::Enum::eCOLLISION_DOWN);

  // override PxUserControllerHitReport in js and assign to
  // PxControllerDesc.reportCallback
  // https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html#deriving-from-c-classes-in-javascript
  class_<PxUserControllerHitReport>("PxUserControllerHitReport")
      .function("onShapeHit", &PxUserControllerHitReport::onShapeHit,
                pure_virtual())
      .function("onControllerHit", &PxUserControllerHitReport::onControllerHit,
                pure_virtual())
      .function("onObstacleHit", &PxUserControllerHitReport::onObstacleHit,
                pure_virtual())
      .allow_subclass<PxUserControllerHitReportWrapper>(
          "PxUserControllerHitReportWrapper");

  class_<PxControllerHit>("PxControllerHit")
      .property("worldPos", &PxControllerHit::worldPos)
      .property("worldNormal", &PxControllerHit::worldNormal)
      .property("dir", &PxControllerHit::dir)
      .property("length", &PxControllerHit::length)
      .function("getCurrentController",
                optional_override(
                    [](PxControllerHit &hit) { return hit.controller; }),
                allow_raw_pointers());

  class_<PxControllerShapeHit, base<PxControllerHit>>("PxControllerShapeHit")
      .function("getTouchedShape",
                optional_override(
                    [](PxControllerShapeHit &hit) { return hit.shape; }),
                allow_raw_pointers())
      .function("getTouchedActor",
                optional_override(
                    [](PxControllerShapeHit &hit) { return hit.actor; }),
                allow_raw_pointers());

  class_<PxControllersHit, base<PxControllerHit>>("PxControllersHit")
      .function(
          "getTouchedController",
          optional_override([](PxControllersHit &hit) { return hit.other; }),
          allow_raw_pointers());
  class_<PxControllerObstacleHit, base<PxControllerHit>>(
      "PxControllerObstacleHit");

  pvdBindings();
}

namespace emscripten {
namespace internal {
// Physx uses private destructors all over the place for its own reference
// counting embind doesn't deal with this well, so we have to override the
// destructors to keep them private in the bindings See:
// https://github.com/emscripten-core/emscripten/issues/5587
template <> void raw_destructor<PxFoundation>(PxFoundation *) { /* do nothing */
}
template <> void raw_destructor<PxMaterial>(PxMaterial *) { /* do nothing */
}
template <> void raw_destructor<PxScene>(PxScene *) { /* do nothing */
}
template <>
void raw_destructor<PxRigidDynamic>(PxRigidDynamic *) { /* do nothing */
}
template <> void raw_destructor<PxRigidBody>(PxRigidBody *) { /* do nothing */
}
template <> void raw_destructor<PxRigidActor>(PxRigidActor *) { /* do nothing */
}
template <> void raw_destructor<PxActor>(PxActor *) { /* do nothing */
}
template <> void raw_destructor<PxShape>(PxShape *) { /* do nothing */
}
template <>
void raw_destructor<PxBVHStructure>(PxBVHStructure *) { /* do nothing */
}
template <>
void raw_destructor<PxRigidStatic>(PxRigidStatic *) { /* do nothing */
}
template <> void raw_destructor<PxJoint>(PxJoint *) { /* do nothing */
}
template <>
void raw_destructor<PxJointLimitParameters>(
    PxJointLimitParameters *) { /* do nothing */
}
template <> void raw_destructor<PxCooking>(PxCooking *) { /* do nothing */
}
template <> void raw_destructor<PxConvexMesh>(PxConvexMesh *) { /* do nothing */
}
template <>
void raw_destructor<PxTriangleMesh>(PxTriangleMesh *) { /* do nothing */
}
template <> void raw_destructor<PxController>(PxController *) { /* do nothing */
}
template <>
void raw_destructor<PxControllerDesc>(PxControllerDesc *) { /* do nothing */
}
template <>
void raw_destructor<PxControllerManager>(
    PxControllerManager *) { /* do nothing */
}
template <>
void raw_destructor<PxHeightField>(PxHeightField *) { /* do nothing */
}
template <>
void raw_destructor<PxCapsuleController>(
    PxCapsuleController *) { /* do nothing */
}
template <>
void raw_destructor<PxBoxController>(PxBoxController *) { /* do nothing */
}
template <>
void raw_destructor<PxUserControllerHitReport>(
    PxUserControllerHitReport *) { /* do nothing */
}

} // namespace internal
} // namespace emscripten
