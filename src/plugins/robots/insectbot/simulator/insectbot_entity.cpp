/**
 * @file <argos3/plugins/robots/kilobot/simulator/insectbot_entity.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "insectbot_entity.h"
#include "kilobot_measures.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/quadrotor_entity.h>

namespace argos
{
   static const Real PROXIMITY_SENSOR_RING_ELEVATION = 0.0001f;
   static const Real PROXIMITY_SENSOR_RING_RADIUS = 0.009f;
   static const CRadians PROXIMITY_SENSOR_RING_START_ANGLE = CRadians((ARGOS_PI / 12.0f) * 0.5f);
   static const Real PROXIMITY_SENSOR_RING_RANGE = 0.08f;
   /****************************************/
   /****************************************/

   CInsectbotEntity::CInsectbotEntity() : CComposableEntity(NULL),
                                      m_pcControllableEntity(NULL),
                                      m_pcEmbodiedEntity(NULL),
                                      m_pcProximitySensorEquippedEntity(NULL),
                                      m_pcQuadrotorEntity(NULL),
                                      m_pcWheeledEntity(NULL),
                                      m_pcPositioningEntity(NULL)
   {
   }

   /****************************************/
   /****************************************/

   CInsectbotEntity::CInsectbotEntity(const std::string &str_id,
                                  const std::string &str_controller_id,
                                  const CVector3 &c_position,
                                  const CQuaternion &c_orientation,
                                  Real f_communication_range) : CComposableEntity(NULL, str_id),
                                                                m_pcControllableEntity(NULL),
                                                                m_pcEmbodiedEntity(NULL),
                                                                m_pcProximitySensorEquippedEntity(NULL),
                                                                m_pcQuadrotorEntity(NULL),
                                                                m_pcWheeledEntity(NULL),
                                                                m_pcPositioningEntity(NULL)
   {
      try
      {
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         this->initEntity();  
         m_pcControllableEntity->SetController(str_controller_id);
         UpdateComponents();


      }
      catch (CARGoSException &ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   void CInsectbotEntity::initEntity(){
         

         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.3f, KILOBOT_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_PIN_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.3f, -KILOBOT_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_PIN_WHEEL_RADIUS);

         m_pcProximitySensorEquippedEntity =
             new CProximitySensorEquippedEntity(this, "proximity_0");
         AddComponent(*m_pcProximitySensorEquippedEntity);
         m_pcProximitySensorEquippedEntity->AddSensorRing(
             CVector3(INSECTBOT_LENGTH/4, 0.0f, PROXIMITY_SENSOR_RING_ELEVATION),
             PROXIMITY_SENSOR_RING_RADIUS,
             PROXIMITY_SENSOR_RING_START_ANGLE,
             PROXIMITY_SENSOR_RING_RANGE,
             24,
             m_pcEmbodiedEntity->GetOriginAnchor());
         
         m_pcQuadrotorEntity = new CQuadRotorEntity(this, "quadrotor_0");
         AddComponent(*m_pcQuadrotorEntity);
         m_pcPositioningEntity = new CPositionalEntity(this, "positioning_0");
         AddComponent(*m_pcPositioningEntity);
         /* Controllable entity.  It must be the last one, for
            actuators/sensors to link to composing entities
            correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
   }


   /****************************************/
   /****************************************/

   void CInsectbotEntity::Init(TConfigurationNode &t_tree)
   {
      try
      {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /* Embodied entity. Better to put this first, because many other entities need this one */
         
         // m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         // AddComponent(*m_pcEmbodiedEntity);
         // m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         // /* Wheeled entity and wheel positions (left, right) */
         // m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         // AddComponent(*m_pcWheeledEntity);
         // m_pcWheeledEntity->SetWheel(0, CVector3(0.3f, KILOBOT_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_PIN_WHEEL_RADIUS);
         // m_pcWheeledEntity->SetWheel(1, CVector3(0.3f, -KILOBOT_HALF_INTERPIN_DISTANCE, 0.0f), KILOBOT_PIN_WHEEL_RADIUS);
         // m_pcProximitySensorEquippedEntity =
         //     new CProximitySensorEquippedEntity(this, "proximity_0");
         // AddComponent(*m_pcProximitySensorEquippedEntity);
         // m_pcProximitySensorEquippedEntity->AddSensorRing(
         //     CVector3(INSECTBOT_LENGTH/4, 0.0f, PROXIMITY_SENSOR_RING_ELEVATION),
         //     PROXIMITY_SENSOR_RING_RADIUS,
         //     PROXIMITY_SENSOR_RING_START_ANGLE,
         //     PROXIMITY_SENSOR_RING_RANGE,
         //     24,
         //     m_pcEmbodiedEntity->GetOriginAnchor());
         // m_pcQuadrotorEntity = new CQuadRotorEntity(this, "quadrotor_0");
         // AddComponent(*m_pcQuadrotorEntity);
         // m_pcPositioningEntity = new CPositionalEntity(this, "positioning_0");
         // AddComponent(*m_pcPositioningEntity);
         // /* Controllable entity. It must be the last one, for
         //    actuators/sensors to link to composing entities
         //    correctly */
         // m_pcControllableEntity = new CControllableEntity(this);
         // AddComponent(*m_pcControllableEntity);
         // m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         // /* Update components */
         // UpdateComponents();
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         this->initEntity();
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         UpdateComponents();
      }
      catch (CARGoSException &ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CInsectbotEntity::Reset()
   {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CInsectbotEntity::Destroy()
   {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT)      \
   if (COMPONENT->IsEnabled()) \
      COMPONENT->Update();

   void CInsectbotEntity::UpdateComponents()
   {
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CInsectbotEntity,
                   "insectbot",
                   "Gilad Yadgar",
                   "1.0",
                   "Insectbot"," The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'body/position' attribute specifies the position of the bottom point of the\n"
                   "Insectbot in the arena. "
                   "The 'body/orientation' attribute specifies the orientation of the robot. All\n"
                   "rotations are performed with respect to the bottom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller/config' attribute is used to assign a controller to the\n"
                   "Insectbot. The value of the attribute must be set to the id of a previously\n"
                   "defined controller. Controllers are defined in the <controllers> XML subtree.\n\n",
                   "Under development");

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CInsectbotEntity);

   /****************************************/
   /****************************************/

} // namespace argos
