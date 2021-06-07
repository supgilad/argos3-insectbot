
#ifndef INSECTBOT_AVOIDER_H
#define INSECTBOT_AVOIDER_H

/*
 * Include some necessary headers.
 */

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/* Random number generator */
#include <argos3/core/utility/math/rng.h>
/* Logging functions */
#include <argos3/core/utility/logging/argos_log.h>
#include <vector>


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;


enum TStateNames {INSECTBOT_STATE_STOP, INSECTBOT_STATE_TURNING, INSECTBOT_STATE_MOVING};


/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CInsectbotAvoider : public CCI_Controller {

public:

   /* Class constructor. */
   CInsectbotAvoider();

   /* Class destructor. */
   virtual ~CInsectbotAvoider() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * These functions allow to track the current state of the robot
    */
   inline const TStateNames GetCurrentState() const {return m_tCurrentState;};

   
private:
   virtual bool isReadingInRange(double reading);
   virtual void log(const std::string& message);


   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcMotors;
   CCI_ProximitySensor* m_sensor;
   CCI_PositioningSensor* m_positionGetter;


   /* behavioural state (moving/turning) */
   TStateNames m_tCurrentState;
   
   /* actual motor speed */
   Real   m_fMotorL;
   Real   m_fMotorR;

   Real m_maxSensorRange;
   Real m_driveSpeed;
   Real m_turnSpeed;
   UInt32  m_stopProb;
   UInt32  m_moveProb;

   UInt32 log_robot_interval;
   UInt32 last_logged_robot;

   /* variables for the random number generation */
   CRandom::CRNG*  m_pcRNG;
   // std::vector<Real>& proximity_reads;
   std::ofstream log_file;
};

#endif
