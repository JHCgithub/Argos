#include "test_1.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>

CFootBotDiffusion::CFootBotDiffusion() :
	m_pcWheels(NULL),
	m_pcProximity(NULL),
	m_cAlpha(10.0f),
	m_fDelta(0.5f),
	m_fWheelVelocity(2.5f),
	m_cGoStraightAngleRange(-ToRadians(m_cAlpha),ToRadians(m_cAlpha)) {}

	void CFootBotDiffusion::Init(TConfigurationNode% t_node){

		m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
		m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

		GetNodeAttirbuteOrDefault(t_node,"alpha",m_cAlpha,m_cAlpha);
		m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
		GetNodeAttirbuteOrDefault(t_node, "delta", m_fDelta,m_fDelta);
		GetNodeAttirbuteOrDefault(t_node,"velocity",m_fWheelVelocity,m_fWheelVelocity);

	}

	void CFootBotDiffusion::ControlStep() {

		const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
		CVector2 cAccumulator;
		for(size_t i = 0; i < tProxReads.size(); ++i){
			cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
		}
		cAccumulator /= tProxReads.size();

		CRadians cAngle = cAccumulator.Angle().
		if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta){
			m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity);
		}else{
			if(cAngle.GetValue() > 0.0f){
				m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
			}else{
				m_pcWheels->SetLinearVelocity(0.0f,m_fWheelVelocity);
			}
		}
	}

	REGISTER_CONTROLLER(CFootBotDiffusion,"footbot_diffusion_controller")