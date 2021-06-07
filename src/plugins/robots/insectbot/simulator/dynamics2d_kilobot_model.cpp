

#include "dynamics2d_kilobot_model.h"
#include "insectbot_measures.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real KILOBOT_MAX_FORCE  = 0.001f;
   static const Real KILOBOT_MAX_TORQUE = 0.001f;
   static const Real KILOBOT_FRICTION   = 1.5f;

   enum KILOBOT_WHEELS {
      KILOBOT_LEFT_WHEEL = 0,
      KILOBOT_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CDynamics2DKilobotModel::CDynamics2DKilobotModel(CDynamics2DEngine& c_engine,
                                                    CInsectbotEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cKilobotEntity(c_entity),
      m_cWheeledEntity(m_cKilobotEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      KILOBOT_MAX_FORCE,
                      KILOBOT_MAX_TORQUE,
                      INSECTBOT_INTERPIN_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Parse the XML file to check if friction was specified */
      cpFloat fFriction = KILOBOT_FRICTION;
      if(c_entity.GetConfigurationNode() &&
         NodeExists(*c_entity.GetConfigurationNode(), "dynamics2d")) {
         TConfigurationNode& tDyn2D = GetNode(*c_entity.GetConfigurationNode(), "dynamics2d");
         GetNodeAttributeOrDefault(tDyn2D, "friction", fFriction, fFriction);
      }
      /* Create the actual body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(INSECTBOT_MASS,
                                  cpMomentForBox(INSECTBOT_MASS,INSECTBOT_BASE_WIDTH,INSECTBOT_BASE_HEIGHT)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the actual body shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpBoxShapeNew(ptBody,INSECTBOT_LENGTH,INSECTBOT_WIDTH));
      ptShape->e = 0.0;       // No elasticity
      ptShape->u = fFriction; // Friction
      /* Constrain the body to follow the diff steering control */
      m_cDiffSteering.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, INSECTBOT_HEIGHT);
   }

   /****************************************/
   /****************************************/

   CDynamics2DKilobotModel::~CDynamics2DKilobotModel() {
      m_cDiffSteering.Detach();
   }

  
   /****************************************/
   /****************************************/

   void CDynamics2DKilobotModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DKilobotModel::UpdateFromEntityStatus() {
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[KILOBOT_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[KILOBOT_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[KILOBOT_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[KILOBOT_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }
   }


   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CInsectbotEntity, CDynamics2DKilobotModel);

   /****************************************/
   /****************************************/

}
