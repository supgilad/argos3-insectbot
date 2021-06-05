/**
 * @file <argos3/plugins/robots/kilobot/simulator/insectbot_entity.h>
 */

#ifndef INSECTBOT_ENTITY_H
#define INSECTBOT_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CInsectbotEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/entities/quadrotor_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

namespace argos {

   class CInsectbotEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CInsectbotEntity();

      CInsectbotEntity(const std::string& str_id,
                     const std::string& str_controller_id,
                     const CVector3& c_position = CVector3(),
                     const CQuaternion& c_orientation = CQuaternion(),
                     Real f_communication_range = 0.1f);
      void initEntity();
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();
      
      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CProximitySensorEquippedEntity& GetProximitySensorEquippedEntity() {
         return *m_pcProximitySensorEquippedEntity;
      }

      inline CQuadRotorEntity& GetQuadRotorEntity() {
         return *m_pcQuadrotorEntity;
      }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "kilobot";
      }

   private:

      CControllableEntity*         m_pcControllableEntity;
      CEmbodiedEntity*             m_pcEmbodiedEntity;
      CWheeledEntity*              m_pcWheeledEntity;
      CQuadRotorEntity*            m_pcQuadrotorEntity;
      CProximitySensorEquippedEntity* m_pcProximitySensorEquippedEntity;
      CPositionalEntity* m_pcPositioningEntity;
   };

}

#endif
