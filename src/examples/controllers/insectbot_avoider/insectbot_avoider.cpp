/* Include the controller definition */
#include "insectbot_avoider.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <time.h>

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
                                         m_positionGetter(NULL),
                                         log_file(),
                                         log_robot_interval(0),
                                         last_logged_robot(0)
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
    */
   // Get sensor/actuator handles
   m_pcMotors = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_sensor = GetSensor<CCI_ProximitySensor>("proximity");
   m_positionGetter = GetSensor<CCI_PositioningSensor>("positioning");
   // Parse the configuration file
   std::string log_file_name;
   GetNodeAttributeOrDefault<std::string>(t_node, "log_file",log_file_name,"experiment.log");
   GetNodeAttributeOrDefault<UInt32>(t_node, "log_robot_interval",log_robot_interval,5);

   if (!log_file.is_open()){
      log_file.open(log_file_name,std::ios_base::app);
      this->log("Robot Created");
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

void CInsectbotAvoider::log(const std::string& message)
{  time_t curr_time;
   curr_time = time(NULL);
   char buffer [80];
   struct tm * timeinfo;
   timeinfo = localtime (&curr_time);
   strftime (buffer,80,"%Y-%m-%d %H:%M:%S",timeinfo);

   const CVector3 &pos = m_positionGetter->GetReading().Position;
   log_file<<R"({"Date":")"<<buffer <<R"(","ID":")"<< GetId()<<R"(","x":")"<<pos[0]<<R"(","y":")"<<pos[1]<<R"(","Message":")"<<message<<R"("})"<<std::endl;
   LOG     <<R"({"Date":")"<<buffer <<R"(","ID":")"<< GetId()<<R"(","x":")"<<pos[0]<<R"(","y":")"<<pos[1]<<R"(","Message":")"<<message<<R"("})"<<std::endl;
}

void CInsectbotAvoider::ControlStep()
{
   time_t seconds;
   seconds = time (NULL);
   if (seconds-last_logged_robot>log_robot_interval){
      this->log("");
      last_logged_robot=seconds;
   }
   
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
         m_fMotorL = PIN_TURN;
         m_fMotorR = PIN_STOP;
      }
      else if (somethingOnFront && !sometingOnRight)
      {
         m_fMotorL = PIN_STOP;
         m_fMotorR = PIN_TURN;
      }
      else if (sometingOnRight && sometingOnLeft)
      {
         m_fMotorL = PIN_STOP;
         m_fMotorR = PIN_TURN;
      }
      else if (sometingOnLeft)
      {
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
