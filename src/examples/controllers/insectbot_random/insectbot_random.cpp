/* Include the controller definition */
#include "insectbot_random.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <vector>
#include <algorithm>

/****************************************/
/****************************************/

CInsectbotRandom::CInsectbotRandom() : m_pcMotors(NULL),
                                       m_sensor(NULL),
                                       m_positionSetter(NULL)
{
   m_pcRNG = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

void CInsectbotRandom::Init(TConfigurationNode &t_node)
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


   Reset();
}

/****************************************/
/****************************************/

void CInsectbotRandom::Reset()
{
}

void CInsectbotRandom::ControlStep()
{
   UInt32 random = m_pcRNG->Uniform(CRange<UInt32>(0, 3));
   if (random < 1)
   {
      m_pcMotors->SetLinearVelocity(400, 400);
   }
   else if (random > 1 && random < 2)
   {
      m_pcMotors->SetLinearVelocity(200, 10);
   }
   else
   {
      m_pcMotors->SetLinearVelocity(10, 60);
   }
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
REGISTER_CONTROLLER(CInsectbotRandom, "insectbot_random_controller")
