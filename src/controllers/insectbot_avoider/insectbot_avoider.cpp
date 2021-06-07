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

#define PIN_STOP 0.0f


CInsectbotAvoider::CInsectbotAvoider() : m_pcMotors(NULL),
                                         m_sensor(NULL),
                                         m_tCurrentState(KILOBOT_STATE_STOP),
                                         m_fMotorL(0.0f),
                                         m_fMotorR(0.0f),
                                         m_positionGetter(NULL),
                                         m_maxSensorRange(0.0f),
                                         m_driveSpeed(0.0f),
                                         m_turnSpeed(0.0f),
                                         m_stopProb(0),
                                         m_moveProb(0),
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
   GetNodeAttributeOrDefault<Real>(t_node, "drive_speed",m_driveSpeed,8.6f);
   GetNodeAttributeOrDefault<Real>(t_node, "turn_speed",m_turnSpeed,2.6f);
   GetNodeAttributeOrDefault<UInt32>(t_node, "stop_uniform_range",m_stopProb,450);
   GetNodeAttributeOrDefault<UInt32>(t_node, "re_move_uniform_range",m_moveProb,200);
   GetNodeAttributeOrDefault<Real>(t_node, "max_range",m_maxSensorRange,1.0f);

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
   m_tCurrentState = KILOBOT_STATE_MOVING;
   m_fMotorL = m_fMotorR = m_driveSpeed;
}

/****************************************/
/****************************************/

bool CInsectbotAvoider::isReadingInRange(double reading)
{
   return (reading > 0.0f && reading < m_maxSensorRange);
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

      UInt32 shouldNotStop = m_pcRNG->Uniform(CRange<UInt32>(0, m_stopProb));
      if (!shouldNotStop)
      {
         m_fMotorL = m_fMotorR = PIN_STOP;
         m_tCurrentState = KILOBOT_STATE_STOP;
      }

      else if (somethingOnFront && !sometingOnLeft)
      {
         m_fMotorL = m_turnSpeed;
         m_fMotorR = PIN_STOP;
      }
      else if (somethingOnFront && !sometingOnRight)
      {
         m_fMotorL = PIN_STOP;
         m_fMotorR = m_turnSpeed;
      }
      else if (sometingOnRight && sometingOnLeft)
      {
         m_fMotorL = PIN_STOP;
         m_fMotorR = m_turnSpeed;
      }
      else if (sometingOnLeft)
      {
         m_fMotorL = PIN_STOP;
         m_fMotorR = m_turnSpeed;
      }
      else if (sometingOnRight)
      {
         m_fMotorL = m_turnSpeed;
         m_fMotorR = PIN_STOP;
      }
      else
      {
         m_fMotorL = m_fMotorR = m_driveSpeed;
      }
   }
   else if (m_tCurrentState == KILOBOT_STATE_STOP)
   {
      UInt32 shouldStayStopped = m_pcRNG->Uniform(CRange<UInt32>(0, m_moveProb));
      if (!shouldStayStopped)
      {
         m_fMotorL = m_fMotorR = m_driveSpeed;
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
