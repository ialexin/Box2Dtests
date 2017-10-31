#pragma once


namespace car {
	enum {
		TDC_LEFT = 0x1,
		TDC_RIGHT = 0x2,
		TDC_UP = 0x4,
		TDC_DOWN = 0x8
	};



	class TDTire {
	public:
		b2Body* m_body;
		float m_maxForwardSpeed;
		float m_maxBackwardSpeed;
		float m_maxDriveForce;
		float rotationCoef = 0.1f;

		TDTire(b2World* world) {
			b2BodyDef bodyDef;
			bodyDef.type = b2_dynamicBody;
			m_body = world->CreateBody(&bodyDef);

			b2PolygonShape polygonShape;
			polygonShape.SetAsBox(0.5f, 1.25f);
			m_body->CreateFixture(&polygonShape, 1);//shape, density

			m_body->SetUserData(this);
		}

		void setCharacteristics(float maxForwardSpeed, float maxBackwardSpeed, float maxDriveForce) {
			m_maxForwardSpeed = maxForwardSpeed;
			m_maxBackwardSpeed = maxBackwardSpeed;
			m_maxDriveForce = maxDriveForce;
		}


		b2Vec2 getLateralVelocity() {
			b2Vec2 currentRightNormal = m_body->GetWorldVector(b2Vec2(1, 0));
			return b2Dot(currentRightNormal, m_body->GetLinearVelocity()) * currentRightNormal;
		}

		b2Vec2 getForwardVelocity() {
			b2Vec2 currentForwardNormal = m_body->GetWorldVector(b2Vec2(0, 1));
			return b2Dot(currentForwardNormal, m_body->GetLinearVelocity()) * currentForwardNormal;
		}

		void updateFriction() {
			//lateral linear velocity
			float maxLateralImpulse = 2.0f;
			b2Vec2 impulse = m_body->GetMass() * -getLateralVelocity();
			if (impulse.Length() > maxLateralImpulse)
				impulse *= maxLateralImpulse / impulse.Length();
			m_body->ApplyLinearImpulse(impulse, m_body->GetWorldCenter(), true);

			//angular velocity
			m_body->ApplyAngularImpulse(0.1f * m_body->GetInertia() * -m_body->GetAngularVelocity(), true);

			//forward linear velocity
			b2Vec2 currentForwardNormal = getForwardVelocity();
			float currentForwardSpeed = currentForwardNormal.Normalize();
			float dragForceMagnitude = -2 * currentForwardSpeed;
			m_body->ApplyForce(dragForceMagnitude * currentForwardNormal, m_body->GetWorldCenter(), true);
		}

		void updateDrive(int controlState) {

			//find desired speed
			float desiredSpeed = 0;
			switch (controlState & (TDC_UP | TDC_DOWN)) {
			case TDC_UP:   desiredSpeed = m_maxForwardSpeed;  break;
			case TDC_DOWN: desiredSpeed = m_maxBackwardSpeed; break;
			default: return;//do nothing
			}

			//find current speed in forward direction
			b2Vec2 currentForwardNormal = m_body->GetWorldVector(b2Vec2(0, 1));
			float currentSpeed = b2Dot(getForwardVelocity(), currentForwardNormal);

			//apply necessary force
			float force = 0;
			if (desiredSpeed > currentSpeed)
				force = m_maxDriveForce;
			else if (desiredSpeed < currentSpeed)
				force = -m_maxDriveForce;
			else
				return;
			m_body->ApplyForce(force * currentForwardNormal, m_body->GetWorldCenter(), true);
		}

		void updateTurn(int controlState) {
			float desiredTorque = 0;
			switch (controlState & (TDC_LEFT | TDC_RIGHT)) {
			case TDC_LEFT:  desiredTorque = 15;  break;
			case TDC_RIGHT: desiredTorque = -15; break;
			default:;//nothing
			}
			m_body->ApplyTorque(desiredTorque, true);
		}

		~TDTire() {
			m_body->GetWorld()->DestroyBody(m_body);
		}
	};

}

using namespace car;

class CarTopDown : public Test {
public:
	car::TDTire tire;
	int m_controlState;

	CarTopDown() : tire(m_world) {
		m_world->SetGravity(b2Vec2(0, 0));
		tire.setCharacteristics(100, -20, 150);
		m_controlState = 0;

	}

	void Keyboard(int key)
	{
		switch (key) {
		case GLFW_KEY_A: m_controlState |= TDC_LEFT; break;
		case GLFW_KEY_D: m_controlState |= TDC_RIGHT; break;
		case GLFW_KEY_W: m_controlState |= TDC_UP; break;
		case GLFW_KEY_S: m_controlState |= TDC_DOWN; break;
		}
	}

	void KeyboardUp(int key)
	{
		switch (key) {
		case GLFW_KEY_A: m_controlState &= ~TDC_LEFT; break;
		case GLFW_KEY_D: m_controlState &= ~TDC_RIGHT; break;
		case GLFW_KEY_W: m_controlState &= ~TDC_UP; break;
		case GLFW_KEY_S: m_controlState &= ~TDC_DOWN; break;
		}
	}

	void Step(Settings* settings)
	{
		tire.updateFriction();
		tire.updateDrive(m_controlState);
		tire.updateTurn(m_controlState);


		//run the default physics and rendering
		Test::Step(settings);

		//show some text in the main screen
		g_debugDraw.DrawString(5, m_textLine, "Now we have a carTopDown test");
		m_textLine += 10;

		
	}

	static Test* Create()
	{
		return new CarTopDown;
	}
};