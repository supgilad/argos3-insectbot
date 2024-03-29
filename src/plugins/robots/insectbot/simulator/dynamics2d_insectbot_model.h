

#ifndef DYNAMICS2D_INSECTBOT_MODEL_H
#define DYNAMICS2D_INSECTBOT_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include "insectbot_entity.h"
namespace argos {

   class CDynamics2DInsectbotModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DInsectbotModel(CDynamics2DEngine& c_engine,
                              CInsectbotEntity& c_entity);
      virtual ~CDynamics2DInsectbotModel();
      
      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      CInsectbotEntity& m_cInsectbotEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;
   };

}

#endif
