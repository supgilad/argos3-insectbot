
#ifndef INSECTBOT_MEASURES_H
#define INSECTBOT_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   static const Real INSECTBOT_INTERPIN_DISTANCE      = 0.025;
   static const Real INSECTBOT_BASE_WIDTH             = 0.01;
   static const Real INSECTBOT_BASE_HEIGHT            = 0.05;
   static const Real INSECTBOT_MASS                   = 0.01;
   static const Real INSECTBOT_LENGTH                   = 0.05;
   static const Real INSECTBOT_WIDTH                  = 0.02;
   static const Real INSECTBOT_HEIGHT                 = 0.02;
   static const Real INSECTBOT_WHEEL_RADIUS       = 0.001;
   static const Real PROXIMITY_SENSOR_RING_ELEVATION = 0.0001f;
   static const Real PROXIMITY_SENSOR_RING_RADIUS = 0.009f;
   static const CRadians PROXIMITY_SENSOR_RING_START_ANGLE = CRadians((ARGOS_PI / 12.0f) * 0.5f);
   static const Real PROXIMITY_SENSOR_RING_RANGE = 0.08f;
   static const Real PROXIMITY_SENSOR_NUM_SENSORS = 24;
}

#endif
