/**
 * @file <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef INSECTBOT_MEASURES_H
#define INSECTBOT_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   /* Kilobot measures */
   static const Real INSECTBOT_INTERPIN_DISTANCE      = 0.025;
   static const Real INSECTBOT_HALF_INTERPIN_DISTANCE = INSECTBOT_INTERPIN_DISTANCE * 0.5;
   static const Real INSECTBOT_BASE_WIDTH             = 0.01;
   static const Real INSECTBOT_BASE_HEIGHT            = 0.05;
   static const Real INSECTBOT_MASS                   = 0.01;
   static const Real INSECTBOT_LENGTH                   = 0.05;
   static const Real INSECTBOT_WIDTH                  = 0.02;
   static const Real INSECTBOT_HEIGHT                 = 0.02;
   static const Real KILOBOT_PIN_WHEEL_RADIUS       = 0.001;
}


#endif
