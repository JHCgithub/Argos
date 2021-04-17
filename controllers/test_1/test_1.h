#ifndef TEST_1_H
#define TEST_1_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_differenital_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

using namespace argos;

class Test1 : public CCI_Controller {
public:
	Test1();
	virtual ~Test1() {}

	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();

	virtual void Reset() {}

	virtual void Destroy() {}
private:
	CCI_DifferentialSterringActuator* m_pcWheels;
	CCI_FootBotProximitySensor* m_pcProximity;

	CDegrees m_cAlpha;

	Real m_fDelta;

	Real m_fWheelVelocity;

	CRange<CRadians> m_cGoStraightAngleRange;



};

#endif