/* Include the controller definition */
#include "insectbot_avoider.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <vector>
#include <algorithm>

/****************************************/
/****************************************/

#define PIN_FORWARD 6.5f
#define PIN_TURN 1.85f
#define PIN_STOP 0.0f
#define STOP_PROB 450
#define MOVE_PROB 200

CInsectbotAvoider::CInsectbotAvoider() : m_pcMotors(NULL),
                                         m_sensor(NULL),
                                         m_tCurrentState(KILOBOT_STATE_STOP),
                                         m_tPreviousState(KILOBOT_STATE_STOP),
                                         m_unMaxMotionSteps(1510),
                                         m_unCountMotionSteps(0),
                                         m_unMaxTurningSteps(90), // = pi/(omega delta_t) = pi/(v*delta_t/l) = (pi*l)/(v*delta_t)
                                         m_unCountTurningSteps(130),
                                         m_fMotorL(0.0f),
                                         m_fMotorR(0.0f),
                                         m_positionSetter(NULL)
{
   m_pcRNG = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void CInsectbotAvoider::Init(TConfigurationNode &t_node)
{
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><kilobot_diffusion><actuators> and
    * <controllers><kilobot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   // Get sensor/actuator handles
   m_pcMotors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_sensor = GetSensor<CCI_ProximitySensor>("proximity");
   m_positionSetter = GetActuator<CQuadRotorPositionDefaultActuator>("quadrotor_position");
   // Parse the configuration file
   GetNodeAttributeOrDefault(t_node, "max_motion_steps", m_unMaxMotionSteps, m_unMaxMotionSteps);
   if (m_unMaxMotionSteps == 0)
   {
      LOGERR << "[FATAL] Invalid value for num_moving_steps (" << m_unMaxMotionSteps << "). Should be a positive integer." << std::endl;
   }

   Reset();
}

/****************************************/
/****************************************/

void CInsectbotAvoider::Reset()
{
   // reset/intialise the robot state
   m_unCountMotionSteps = m_pcRNG->Uniform(CRange<UInt32>(1, m_unMaxMotionSteps + 1));
   m_tCurrentState = KILOBOT_STATE_MOVING;
   m_tPreviousState = KILOBOT_STATE_MOVING;
   m_fMotorL = m_fMotorR = PIN_FORWARD;
}

/****************************************/
/****************************************/

static bool isReadingInRange(double reading, double min = 0.0f, double max = 1.0f)
{
   return (reading > min && reading < max);
}

void CInsectbotAvoider::ControlStep()
{
   
   const std::vector<double> &tProxReads = m_sensor->GetReadings();
   if (m_tCurrentState == KILOBOT_STATE_MOVING)
   {
      const double sometingOnRight = isReadingInRange(std::min(tProxReads[2], tProxReads[1])) ||
                                     isReadingInRange(std::min(tProxReads[3], tProxReads[4]));
      const double sometingOnLeft = isReadingInRange(std::min(tProxReads[22], tProxReads[21])) ||
                                    isReadingInRange(std::min(tProxReads[20], tProxReads[19]));
      const double somethingOnFront = isReadingInRange(std::min(tProxReads[0], tProxReads[23]));

      UInt32 shouldNotStop = m_pcRNG->Uniform(CRange<UInt32>(0, STOP_PROB));
      if (!shouldNotStop)
      {
         m_fMotorL = m_fMotorR = PIN_STOP;
         m_tCurrentState = KILOBOT_STATE_STOP;
      }

      else if (somethingOnFront && !sometingOnLeft)
      {
         // std::cout << "dist is" << std::min(tProxReads[0], tProxReads[23]);
         // std::cout << "dist is" << std::min(tProxReads[0], tProxReads[23]);
         RLOG << m_pcRNG->Uniform(CRange<UInt32>(0, STOP_PROB))<<"," <<m_pcRNG->Uniform(CRange<UInt32>(0, STOP_PROB)) <<std::endl;
         m_fMotorL = PIN_TURN;
         m_fMotorR = PIN_STOP;
      }
      else if (somethingOnFront && !sometingOnRight)
      {
         // std::cout <<"dist is" << std::min(tProxReads[0], tProxReads[23]);

         m_fMotorL = PIN_STOP;
         m_fMotorR = PIN_TURN;
      }
      else if (sometingOnRight && sometingOnLeft)
      {
         // std::cout <<"dist is" << std::min(tProxReads[1], tProxReads[22]);

         m_fMotorL = PIN_STOP;
         m_fMotorR = PIN_TURN;
      }
      else if (sometingOnLeft)
      {
         std::cout<<std::min(tProxReads[0], tProxReads[23]);
         m_fMotorL = PIN_STOP;
         m_fMotorR = PIN_TURN;
      }
      else if (sometingOnRight)
      {
         m_fMotorL = PIN_TURN;
         m_fMotorR = PIN_STOP;
      }
      else
      {
         m_fMotorL = m_fMotorR = PIN_FORWARD;
      }
   }
   else if (m_tCurrentState == KILOBOT_STATE_STOP)
   {
      UInt32 shouldStayStopped = m_pcRNG->Uniform(CRange<UInt32>(0, MOVE_PROB));
      if (!shouldStayStopped)
      {
         m_fMotorL = m_fMotorR = PIN_FORWARD;
         m_tCurrentState = KILOBOT_STATE_MOVING;
      }
      else
      {
         m_fMotorL = m_fMotorR = PIN_STOP;
      }
   };

   m_pcMotors->SetLinearVelocity(m_fMotorL, m_fMotorR);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.  The string is then usable in the configuration
 * file to refer to this controller.  When ARGoS reads that string in
 * the configuration file, it knows which controller class to
 * instantiate.  See also the configuration files for an example of
 * how this is used.
 */
REGISTER_CONTROLLER(CInsectbotAvoider, "insectbot_avoider_controller")
