#pragma once

#include "BasicActors.h"
#include "VisualDebugger.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {
		PxVec3( 46.0f / 255.0f,   9.0f / 255.0f,  39.0f / 255.0f), // Purple
		PxVec3(217.0f / 255.0f,   0.0f / 255.0f,   0.0f / 255.0f), // Watermelon
		PxVec3(255.0f / 255.0f,  45.0f / 255.0f,   0.0f / 255.0f), // Orange
		PxVec3(255.0f / 255.0f, 140.0f / 255.0f,  54.0f / 255.0f), // Yellow
		PxVec3(  4.0f / 255.0f, 117.0f / 255.0f, 111.0f / 255.0f), // Teal
		PxVec3(0, 0.5, 0) // Grass
	};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			CAR = (1 << 0),
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		//if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		//{
			//pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			//return PxFilterFlags();
		//}

		//pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			return PxFilterFlag::eSUPPRESS;
		}

		return PxDefaultSimulationFilterShader(
			attributes0, filterData0,
			attributes1, filterData1,
			pairFlags, constantBlock, constantBlockSize
		);
	};


	/// <summary>
	/// Represents the gears the car has.
	/// </summary>
	enum class CarGear {
		Forward, // Accelerate until hitting target speed.
		Neutral, // No force applied.
		Backward // Accelerate until hitting negative target speed.
	};

	/// <summary>
	/// Represents whether or not the car is turning.
	/// </summary>
	enum class WheelRotation {
		Left,     // Rotate wheel left until hitting limit.
		Neutral,  // No rotation applied
		Right,    // Rotate wheel right until hitting limit.
	};

	/// <summary>
	/// Container for car behaviour.
	/// </summary>
	class Car
	{
		DynamicActor* body;
		DynamicActor* wheels[4];
		PxVec3 wheelOffsets[4]{
			PxVec3{  1, -0.25,  1.5 },
			PxVec3{ -1, -0.25,  1.5 },
			PxVec3{  1, -0.25, -1.5 },
			PxVec3{ -1, -0.25, -1.5 },
		};
		DistanceJoint* wheelSprings[4];
		RevoluteJoint* wheelMotors[4];
		CarGear gear{ CarGear::Neutral };
		PxReal targetSpeed{ 50 };
		PxReal acceleration{ 10 };
		PxReal deacceleration{ 20 };
		WheelRotation wheelRotation{ WheelRotation::Neutral };
		PxReal wheelAngleLimit{ PxPi / 4 }; // 45 degrees
		PxReal rotationSpeed{ 10 };
		

		/// <summary>
		/// Applies force appropriately to reach target speed if necessary.
		/// </summary>
		/// <param name="deltaTime">Time passed since last update</param>
		void UpdateForce(PxReal deltaTime) {
			PxReal speed = GetSpeed();
			switch (gear)
			{
			case CarGear::Forward:
				SetSpeed(speed + acceleration * deltaTime);
				break;
			case CarGear::Neutral:
				SetSpeed(PxClamp<PxReal>(speed - deacceleration * deltaTime, 0, 66535));
				break;
			case CarGear::Backward:
				SetSpeed(speed - acceleration * deltaTime);
				break;
			default:
				break;
			}
		}

		void UpdateWheelAngle(PxReal deltaTime) {
			PxReal angle = GetWheelAngle();
			cout << angle << endl;
			switch (wheelRotation)
			{
			case WheelRotation::Left:
				SetWheelAngle(angle + rotationSpeed * deltaTime);
				break;
			case WheelRotation::Neutral:
				SetWheelAngle(PxClamp<PxReal>(angle - rotationSpeed * deltaTime, 0, 2 * PxPi));
				break;
			case WheelRotation::Right:
				SetWheelAngle(angle - rotationSpeed * deltaTime);
				break;
			default:
				break;
			}
		}

	public:
		PxVec3 cameraOffset{ 0, 1, -3 };

		Car(Scene& scene, physx::PxTransform transform) {
			// Body
			body = new CarBody{ transform };
			body->Color(color_palette[4]); // Teal
			body->Name("CarBody");
			auto material = GetPhysics()->createMaterial(0.5, 0.5, 0.65);
			body->Material(material);
			body->SetupFiltering(FilterGroup::CAR, FilterGroup::CAR);
			scene.Add(body);
			// Wheels
			material = GetPhysics()->createMaterial(0.8, 0.8, 0.65);
			for (int wheelNumber = 0; wheelNumber < 4; wheelNumber++)
			{
				// Wheels actor
				wheels[wheelNumber] = new Wheel{
					0.3,
					0.4,
					PxTransform{
						transform.p + wheelOffsets[wheelNumber],
						transform.q
					},
				};
				wheels[wheelNumber]->Name("CarWheel" + wheelNumber);
				wheels[wheelNumber]->Material(material);
				wheels[wheelNumber]->SetupFiltering(FilterGroup::CAR, FilterGroup::CAR);
				// Wheels suspension
				//wheelSprings[wheelNumber] = new DistanceJoint(
					//body,
					//PxTransform{ wheelOffsets[wheelNumber] },
					//wheels[wheelNumber],
					//PxTransform{ 0, 0, 0 }
				//);
				//wheelSprings[wheelNumber]->Damping(1'000);
				//wheelSprings[wheelNumber]->Stiffness(30'000);
				// Wheel motors
				wheelMotors[wheelNumber] = new RevoluteJoint{
					body,
					PxTransform{ wheelOffsets[wheelNumber], transform.q },
					wheels[wheelNumber],
					PxTransform{ 0, 0, 0 }
				};
				scene.Add(wheels[wheelNumber]);
			}
		}
		~Car()
		{
			body->~DynamicActor();
		}

		void Update(PxReal deltaTime) {
			UpdateForce(deltaTime);
			UpdateWheelAngle(deltaTime);
		}

		/// <summary>
		/// Gets the actor associated with the body of the car. This is, in effect, the primary part of the car used for determination of position and rotation.
		/// </summary>
		/// <returns>A pointer to the RigidDynamic actor instance.</returns>
		PxRigidDynamic* GetActor() {
			return body->GetDynamic();
		}

		/// <summary>
		/// Get method for the car's gear.
		/// </summary>
		/// <returns>Current car gear</returns>
		CarGear GetGear() {
			return gear;
		}

		/// <summary>
		/// Set method for the car's gear.
		/// </summary>
		/// <param name="newGear">New car gear</param>
		void SetGear(CarGear newGear) {
			gear = newGear;
		}

		/// <summary>
		/// Gets the current velocity of the car's wheels.
		/// </summary>
		/// <returns>The current velocity of the car's wheels.</returns>
		PxReal GetSpeed() {
			return wheelMotors[0]->DriveVelocity();
		}

		/// <summary>
		/// Sets the velocity of the car's wheels.
		/// </summary>
		/// <param name="velocity">The new velocity of the car's wheels</param>
		void SetSpeed(PxReal velocity) {
			velocity = PxClamp(velocity, -targetSpeed, targetSpeed);
			for each (RevoluteJoint* motor in wheelMotors) {
				motor->DriveVelocity(velocity);
			}
		}

		/// <summary>
		/// Get method for the wheel's rotation enum.
		/// </summary>
		/// <returns>Wheel rotation for the vehicle</returns>
		WheelRotation GetWheelRotation() {
			return wheelRotation;
		}

		/// <summary>
		/// Set method for the wheel's rotation enum
		/// </summary>
		/// <param name="newWheelRotation">Wheel rotation for the vehicle</param>
		void SetWheelRotation(WheelRotation newWheelRotation) {
			wheelRotation = newWheelRotation;
		}

		/// <summary>
		/// Get method for the wheels' current angle
		/// </summary>
		/// <returns>Returns an angle in radians</returns>
		PxReal GetWheelAngle() {
			return wheelMotors[0]->Get()->getLocalPose(PxJointActorIndex::eACTOR0).q.y;
		}

		/// <summary>
		/// Set method for the wheels' current angle
		/// </summary>
		/// <param name="newAngle">The new wheel angle (in radians)</param>
		void SetWheelAngle(PxReal newAngle) {
			newAngle = PxClamp(newAngle, -wheelAngleLimit, wheelAngleLimit);
			for (size_t wheelNumber = 0; wheelNumber < 2; wheelNumber++) // Only get the front wheels
			{
				wheelMotors[wheelNumber]->Get()->setLocalPose(
					PxJointActorIndex::eACTOR0,
					PxTransform{
						wheelOffsets[wheelNumber],
						PxQuat{
							newAngle,
							wheelMotors[wheelNumber]->Get()->getLocalPose(PxJointActorIndex::eACTOR0).q.getBasisVector1()
						}
					}
				);
			}
		}
	};



	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		
		MySimulationEventCallback* my_callback;
		
	public:
		Car* car;
		Sphere* ball;

		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			auto planeMaterial = GetPhysics()->createMaterial(0.5, 0.4, 0.2);
			plane->Material(planeMaterial);
			plane->Color(color_palette[5]);
			Add(plane);

			// Create ball
			ball = new Sphere(
				PxTransform{ PxVec3{ 0.0, 0.5, 0.5 } },
				0.5,
				1);
			ball->Color(color_palette[2]); // Watermelon
			ball->Name("Ball");
			Add(ball);

			car = new Car(
				*this,
				PxTransform {
					PxVec3 { 0, 1.5, -5 }
				}
			);

			//set collision filter flags
			// box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1);
			//use | operator to combine more actors e.g.
			// box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);
			//don't forget to set your flags for the matching actor as well, e.g.:
			// box2->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);

			/*
			//joint two boxes together
			//the joint is fixed to the centre of the first box, oriented by 90 degrees around the Y axis
			//and has the second object attached 5 meters away along the Y axis from the first object.
			RevoluteJoint joint(box, PxTransform(PxVec3(0.f,0.f,0.f),PxQuat(PxPi/2,PxVec3(0.f,1.f,0.f))), box2, PxTransform(PxVec3(0.f,5.f,0.f)));
			*/
		}

		//Custom udpate function
		virtual void CustomUpdate(PxReal dt) 
		{
			car->Update(dt);
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}
	};
}
