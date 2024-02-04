/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );

   m_pcPosSens   = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   //m_cOutput.open("output.txt", std::ios::app);
   //m_cOutput << m_pcPosSens->GetReading().Position;

   GetNodeAttribute(t_node, "x", x_path);
   GetNodeAttribute(t_node, "y", y_path);
   GetNodeAttribute(t_node, "x2", x2_path);
   GetNodeAttribute(t_node, "y2", y2_path);
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   Reset();
}

void CFootBotDiffusion::Reset(){
  m_eState = STATE_ROTATING;
}



/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vecUInt32tor is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   m_cOutput.close();
   CVector3 pos = m_pcPosSens->GetReading().Position;
   CQuaternion quat = m_pcPosSens->GetReading().Orientation;

   CRadians yaw, temp1, temp2;

   switch(m_eState) {
     case STATE_ROTATING: {
       LOGERR << "rotate" << std::endl;
       quat.ToEulerAngles(yaw, temp1, temp2);
       Rotate(x_path-pos[0], y_path-pos[1], yaw);
       break;
     }
     case STATE_DRIVE: {
       double dist = sqrt(pow(x_path-pos[0],2)+pow(y_path-pos[1],2));
       LOGERR << "drive" << dist << "pos" << pos[0] << "," << pos[1] << std::endl;
       Drive(dist);
       break;
     }
     case STATE_NEW_POINT: {
       LOGERR << "stop" << std::endl;
       x_path = x2_path;
       y_path = y2_path;
       m_eState = STATE_ROTATING;
       break;
     }
     default: {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
     }
   /*
   else {   m_cOutput << "rotating" << std::endl;

     m_cOutput << "3" << std::endl;
     if(sqrt(pow(pos[1]-sety,2)+pow(pos[1]-sety,2) > 0.1)){
       m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
     }
     else{
       m_pcWheels->SetLinearVelocity(0, 0);
     }

   }
   */
 }
}

void CFootBotDiffusion::Rotate(double x, double y, argos::CRadians yaw){
    double angleerr = 0;
    angleerr =  atan2(y, x) - yaw.GetValue();
    LOGERR << angleerr << std::endl;

    if(abs(angleerr) > 0.05){
      double kp = 1;
      if(angleerr<0){
        m_pcWheels->SetLinearVelocity(kp*m_fWheelVelocity, -kp*m_fWheelVelocity);
      }
      else{
        m_pcWheels->SetLinearVelocity(-kp*m_fWheelVelocity, kp*m_fWheelVelocity);
      }
    }
    else{
      m_pcWheels->SetLinearVelocity(0, 0);
      m_eState = STATE_DRIVE;
    }
}

void CFootBotDiffusion::Drive(double distance){
    if(distance > 0.03 || distance < -0.03){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
    else {
      m_pcWheels->SetLinearVelocity(0, 0);
      m_eState = STATE_NEW_POINT;
    }
}
/*
void CFootBotDiffusion::Rotate(double distance){

}
*/

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
