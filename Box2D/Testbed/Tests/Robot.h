#pragma once

namespace car2 {
	float multiplier = 10;

	class TDTire {
	public:
		b2Body* m_body;
		float m_maxForwardSpeed;
		float m_maxBackwardSpeed;
		float m_maxDriveForce;
		float rotationCoef = 0.1f;

		TDTire(b2World* world, b2Vec2 &pos) {
			b2BodyDef bodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = pos;

			m_body = world->CreateBody(&bodyDef);

			b2FixtureDef fix;
			b2PolygonShape polygonShape;
			polygonShape.SetAsBox(0.056f / 2.0f * multiplier, 0.028f / 2.0f * multiplier);
			fix.shape = &polygonShape;
			fix.restitution = 0;
			fix.density = 1.0f;
			m_body->CreateFixture(&fix);//shape, density

			m_body->SetUserData(this);
		}

		b2Vec2 getLateralVelocity() {
			/////ÈÑÏÐÀÂÈË Â ÝÒÎÉ ÑÒÐÎ×ÊÅ!!!
			b2Vec2 currentRightNormal = m_body->GetWorldVector(b2Vec2(0, 1));
			return b2Dot(currentRightNormal, m_body->GetLinearVelocity()) * currentRightNormal;
		}

		b2Vec2 getForwardVelocity() {
			b2Vec2 currentForwardNormal = m_body->GetWorldVector(b2Vec2(1, 0));
			return b2Dot(currentForwardNormal, m_body->GetLinearVelocity()) * currentForwardNormal;
		}

		void keepConstantSpeed(float speed) {
			b2Vec2 lateralImpulse = 1 * m_body->GetMass() * -getLateralVelocity();
			m_body->ApplyLinearImpulse(lateralImpulse, m_body->GetWorldCenter(), true);

			b2Vec2 forwardNormal = getForwardVelocity();
			float scalar = b2Dot(forwardNormal, m_body->GetWorldVector(b2Vec2(1, 0))) < 0 ? -1 : 1;
			float currentForwardSpeed = forwardNormal.Normalize() * scalar;
			forwardNormal = m_body->GetWorldVector(b2Vec2(1, 0));

			float desiredSpeed = currentForwardSpeed;
			float speedPiece = 0.01f;
			if (currentForwardSpeed < speed) {
				desiredSpeed += speedPiece;
			}
			else if (currentForwardSpeed > speed) {
				desiredSpeed -= speedPiece;
			}
			else return;
			
			float speedDiff = desiredSpeed - currentForwardSpeed;
			b2Vec2 linearImpulse = speedDiff * m_body->GetMass() * forwardNormal;
			m_body->ApplyLinearImpulseToCenter(linearImpulse, true);

			/*float dragForce = -0.001 * currentForwardSpeed;
			m_body->ApplyForceToCenter(dragForce * forwardNormal, true);*/

			/*if (currentForwardSpeed * speed < 0) {
				
			}

			
			float speedDiff = speed - currentForwardSpeed;
			b2Vec2 linearImpulse = speedDiff * m_body->GetMass() * forwardNormal;
			m_body->ApplyLinearImpulseToCenter(linearImpulse, true);
*/
			/*float maxForce = 0.001 * multiplier;
			float force = 0;
			if (currentForwardSpeed < speed) {
				force = maxForce;
			}
			else if (currentForwardSpeed > speed) {
				force = -maxForce;
			}
			else
				return;
			m_body->ApplyForceToCenter(force * forwardNormal, true);*/
			////lateral linear velocity
			//b2Vec2 impulse = m_body->GetMass() * -getLateralVelocity();
			//m_body->ApplyLinearImpulseToCenter(impulse, true);

			////forward linear velocity
			//b2Vec2 currentForwardNormal = getForwardVelocity();
			//float currentForwardSpeed = currentForwardNormal.Normalize();
			//float speedChange = currentForwardSpeed - speed;
			//b2Vec2 forwardImpulse = speedChange * m_body->GetMass() * -currentForwardNormal;
			//m_body->ApplyLinearImpulseToCenter(forwardImpulse, true);
		}

		~TDTire() {
			m_body->GetWorld()->DestroyBody(m_body);
		}
	};

}

using namespace car2;
class Robot : public Test {
public:
	car2::TDTire leftWheel;
	car2::TDTire rightWheel;
	b2Body *robot;
	b2RevoluteJoint *leftJoint, *rightJoint;
	bool isStart = true;

	Robot() : leftWheel(m_world, multiplier * b2Vec2(0, 0)), rightWheel(m_world, multiplier * b2Vec2(0, -0.175f)) {
		m_world->SetGravity(b2Vec2(0, 0));

		//Robot
		{
			b2BodyDef bodyDef;
			bodyDef.position = multiplier * b2Vec2(0, -0.0875f);
			bodyDef.angle = 0;
			bodyDef.type = b2_dynamicBody;

			robot = m_world->CreateBody(&bodyDef);

			b2FixtureDef robotFixture;
			b2PolygonShape polygonShape;
			b2Vec2 *points = new b2Vec2[4];
			points[0] = multiplier * b2Vec2(0, 0.0875f);
			points[1] = multiplier * b2Vec2(0.175f, 0.0875f);
			points[2] = multiplier * b2Vec2(0.175f, -0.0875f);
			points[3] = multiplier * b2Vec2(0, -0.0875f);
			//polygonShape.SetAsBox(0.175f / 2.0f * multiplier, 0.175f / 2.0f * multiplier/*, multiplier * b2Vec2(0.0875f, 0), 0*/);
			polygonShape.Set(points, 4);

			robotFixture.shape = &polygonShape;
			robotFixture.density = 0.01f;//1.05f / (0.175f * 0.175f);
			robotFixture.restitution = 0;
			robotFixture.friction = 1;

			robot->CreateFixture(&robotFixture);
			
			/*b2FixtureDef heavyRect;
			b2PolygonShape pShape;
			pShape.SetAsBox(0.005f * multiplier, 0.005f * multiplier);
			heavyRect.shape = &pShape;
			heavyRect.density = 0.5f / (0.01f * 0.01f);
			heavyRect.friction = 1;

			robot->CreateFixture(&heavyRect);

			b2FixtureDef heavyRect2;
			b2PolygonShape pShape2;
			pShape2.SetAsBox(0.005f * multiplier, 0.005f * multiplier);
			heavyRect2.shape = &pShape2;
			heavyRect2.density = 0.5f / (0.01f * 0.01f);
			heavyRect2.friction = 1;

			robot->CreateFixture(&heavyRect2);*/
		}
		
		//leftWheel
		{
			b2RevoluteJointDef revDef;
				
			//leftWheel.m_body->SetTransform(b2Vec2(-3.0f, -1.0f), 0);
			revDef.bodyA = leftWheel.m_body;
			revDef.bodyB = robot;
			revDef.collideConnected = false;

			revDef.localAnchorA = leftWheel.m_body->GetLocalCenter();
			////ÈÑÏÐÀÂÈÒÜ ÝÒÎ!!!!!
			revDef.localAnchorB = robot->GetLocalPoint(leftWheel.m_body->GetWorldCenter());//b2Vec2(-0.015f / 2.0f, 0);

			revDef.referenceAngle = 0;
			revDef.enableLimit = true;
			revDef.lowerAngle = 0;
			revDef.upperAngle = 0;

			m_world->CreateJoint(&revDef);
		}

		//rightWheel
		{
			b2RevoluteJointDef revDef;

			//rightWheel.m_body->SetTransform(b2Vec2(3.0f, -1.0f), 0);
			revDef.bodyA = rightWheel.m_body;
			revDef.bodyB = robot;
			revDef.collideConnected = false;

			revDef.localAnchorA = rightWheel.m_body->GetLocalCenter();
			revDef.localAnchorB = robot->GetLocalPoint(rightWheel.m_body->GetWorldCenter());

			revDef.referenceAngle = 0;
			revDef.enableLimit = true;
			revDef.lowerAngle = 0;
			revDef.upperAngle = 0;

			m_world->CreateJoint(&revDef);
		}

		//wall
		{
			b2BodyDef bodyDef;
			bodyDef.position = multiplier * b2Vec2(0.175f, 0) + b2Vec2(0.7, 0);
			bodyDef.type = b2_staticBody;
			b2Body *wall = m_world->CreateBody(&bodyDef);

			b2FixtureDef def;
			b2PolygonShape shape;
			shape.SetAsBox(0.3, multiplier * 2);
			def.shape = &shape;
			def.restitution = 0;
			wall->CreateFixture(&def);
		}
	}

	static Test* Create()
	{
		return new Robot;
	}

	float speed = 0;

	void Step(Settings* settings)
	{
		if (isStart) {
			robot->ApplyForceToCenter(b2Vec2(0.00001f * multiplier, 0), true);
			isStart = false;
		}
		float hz = settings->hz;

		//leftWheel.m_body->SetLinearVelocity(leftWheel.m_body->GetWorldVector(b2Vec2(0.0f, 0)));
		//rightWheel.m_body->SetLinearVelocity(rightWheel.m_body->GetWorldVector(b2Vec2(0.02695f * multiplier, 0)));
		leftWheel.keepConstantSpeed(speed * multiplier);
		rightWheel.keepConstantSpeed(speed * multiplier);

		//run the default physics and rendering
		Test::Step(settings);

		//show some text in the main screen
		g_debugDraw.DrawString(5, m_textLine, "Now we have a robot test");
		m_textLine += 10;
		g_debugDraw.DrawString(5, m_textLine, "%f", leftWheel.m_body->GetLinearVelocity().Length());
		m_textLine += 10;
		g_debugDraw.DrawString(5, m_textLine, "%f", rightWheel.m_body->GetLinearVelocity().Length());
		m_textLine += 10;
		b2MassData *data = new b2MassData();
		robot->GetMassData(data);
		g_debugDraw.DrawString(5, m_textLine, "%f", data->center.x);
		m_textLine += 10;
		g_debugDraw.DrawString(5, m_textLine, "%f", data->center.y);
		m_textLine += 10;
		g_debugDraw.DrawString(5, m_textLine, "%f", robot->GetPosition().x);
		m_textLine += 10;
		g_debugDraw.DrawString(5, m_textLine, "%f", robot->GetPosition().y);
		m_textLine += 10;
		g_debugDraw.DrawString(5, m_textLine, "%f", robot->GetMass());
	}

	void Keyboard(int key)
	{
		float speedDiff = 0.1f;
		switch (key)
		{
		case GLFW_KEY_A:
		{
			speed -= speedDiff;
		}
		break;

		case GLFW_KEY_D:
		{
			speed += speedDiff;
		}
		break;
		}
	}
};