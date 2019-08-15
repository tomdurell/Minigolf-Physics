#pragma once

#include "BasicActors.h"
#include <iostream>  
#include <string>    
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;
	
	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = { PxVec3(46.f / 255.f,9.f / 255.f,39.f / 255.f),PxVec3(217.f / 255.f,0.f / 255.f,0.f / 255.f),
		PxVec3(255.f / 255.f,45.f / 255.f,0.f / 255.f),PxVec3(255.f / 255.f,140.f / 255.f,54.f / 255.f),PxVec3(4.f / 255.f,117.f / 255.f,111.f / 255.f) };

	//pyramid vertices
	static PxVec3 pyramid_verts[] = { PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1) };
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = { 1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1 };

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose = PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs), end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0 = (1 << 0),
			ACTOR1 = (1 << 1),
			ACTOR2 = (1 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions = PxVec3(1.f, 1.f, 1.f), PxReal stiffness = 1.f, PxReal damping = 1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f, thickness, 0.f)), PxVec3(dimensions.x, thickness, dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f, dimensions.y + thickness, 0.f)), PxVec3(dimensions.x, thickness, dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x, thickness, dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -dimensions.y, dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x, thickness, -dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -dimensions.y, -dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x, thickness, dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -dimensions.y, dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x, thickness, -dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -dimensions.y, -dimensions.z)));

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
		bool winCondition = false;
		PxRigidDynamic* resetBallForce;
		//an example variable that will be checked in the main simulation loop
		bool trigger;
		//ballOrigin.p = PxVec3(0.f,2.f,-45.f);

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
						
						//otherShape->setGlobalPose(ballOrigin);
						pairs[i].otherActor->setGlobalPose(PxTransform(PxVec3(0.f, 2.f, -45.f)));
						resetBallForce = (PxRigidDynamic*)pairs[i].otherActor->isRigidDynamic();
						resetBallForce->putToSleep();

						if (std::strcmp(pairs[i].triggerActor->getName() , "Goal") ==0)
						{
							 winCondition = true;
						}

						//is asleep? forces continue to act after the teleport, so set momentum etc to 0;
						//pairs[i].otherActor->
						
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
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		// let triggers through
		if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
		//		pairFlags |= PxPairFlag::eCCD_LINEAR;


		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{

		

	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};
		MySimulationEventCallback* my_callback;
		
		Cloth* cloth;
		Box* box, *box2, *box3, *box4, *box5, *box6, *icePlatform, *outofBounds, *goalBox, *flagpole, *winBox;
		Slider* slidingDoor;
		Bumper* bumpSet1, *bumpSet2, *bumpSet3, *bumpSet4, *bumpSet5, *bumpSet6, *bumpSet7, *bumpSet8, *bumpSet9;
		PxRigidActor* actor, *wallActor, *planeActor;
		startPlatform* originPlatform;
		Platform* windmillPlatform;
		cornerPlatform* iceCorner;
		Sphere* golfBall;
		//Sphere* golfBalls [2000];
		Ramp* ramp1, *ramp2; //*testBox;
		PxRigidDynamic* golfballRigid;
		
		Hole* goalHole;

		bool wallUp = true;
		bool wallDown = false;
		
		
		

		float golfBallRadius = 1.f;
		//float debugCounter = 0.0f;
		//int spawnCounter = 0;
		//int golfSpawnX = 0;
		//int golfSpawnY = 0;
		float sliderDoorController = 0.0f;
		float bumpRadius = 1.0f;


		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);

		}

		//Custom scene initialisation
		virtual void CustomInit()
		{
			

			SetVisualisation();
			GetMaterial()->setDynamicFriction(.2f);
			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);


			PxMaterial* bumperMat = CreateMaterial(0.f, 0.f, 1.f);
			PxMaterial* woodMat = CreateMaterial(0.3f, 0.2f, 0.3f);
			PxMaterial* iceMat = CreateMaterial(0.f, 0.f, .5f);

			outofBounds = new Box(PxTransform(PxVec3(.0f, -30.f, 0.f)), PxVec3(500.f,1.f,500.f));
			outofBounds->Color(color_palette[4]);
			outofBounds->SetTrigger(1);
			outofBounds->SetKinematic(1);
			Add(outofBounds);
			
			winBox = new Box(PxTransform(PxVec3(-96.5f, -6.f, 194.f)), PxVec3(2.0f, 2.0f, 2.0f));
			winBox->Color(color_palette[2]);
			winBox->Name("Goal");
			winBox->SetTrigger(1);
			winBox->SetKinematic(1);
			Add(winBox);

			originPlatform = new startPlatform(PxTransform(PxVec3(.0f, 0.f, .0f)), PxVec3(10.f, 10.f, 50.0f));
			originPlatform->Color(color_palette[1]);
			originPlatform->Material(woodMat);
			Add(originPlatform);


			PxQuat iceCornerRot = PxQuat(-1.57f, PxVec3(0.f, 1.f, 0.f));
			iceCorner = new cornerPlatform(PxTransform(PxVec3(-38.f, -2.f, 195.f), iceCornerRot), PxVec3(10.f, 10.f, 50.0f));
			iceCorner->Color(color_palette[4]);
			iceCorner->Material(iceMat);
			Add(iceCorner);

			bumpSet1 = new Bumper(PxTransform(PxVec3(0.f, 1.f, 0.f)), bumpRadius);
			Add(bumpSet1);
			bumpSet1->Material(bumperMat);

			bumpSet2 = new Bumper(PxTransform(PxVec3(6.f, 1.f, 0.f)), bumpRadius);
			bumpSet2->Material(bumperMat);
			Add(bumpSet2);

			bumpSet3 = new Bumper(PxTransform(PxVec3(-6.f, 1.f, 0.f)), bumpRadius);
			Add(bumpSet3);
			bumpSet3->Material(bumperMat);

			bumpSet4 = new Bumper(PxTransform(PxVec3(-6.f, 1.f, 3.f)), bumpRadius);
			Add(bumpSet4);
			bumpSet4->Material(bumperMat);

			bumpSet5 = new Bumper(PxTransform(PxVec3(6.f, 1.f, 3.f)), bumpRadius);
			Add(bumpSet5);
			bumpSet5->Material(bumperMat);

			bumpSet6 = new Bumper(PxTransform(PxVec3(0.f, 1.f, 3.f)), bumpRadius);
			Add(bumpSet6);
			bumpSet6->Material(bumperMat);

			bumpSet7 = new Bumper(PxTransform(PxVec3(8.f, 1.f, 6.f)), bumpRadius);
			Add(bumpSet7);
			bumpSet7->Material(bumperMat);

			bumpSet8 = new Bumper(PxTransform(PxVec3(0.f, 1.f, 6.f)), bumpRadius);
			Add(bumpSet8);
			bumpSet8->Material(bumperMat);

			bumpSet9 = new Bumper(PxTransform(PxVec3(-8.f, 1.f, 6.f)), bumpRadius);
			Add(bumpSet9);
			bumpSet9->Material(bumperMat);

			goalHole = new Hole(PxTransform(PxVec3(-90.5f, -2.f, 194.f)), PxVec3(5.f, 5.f, 5.f));
			Add(goalHole);

			/*testBox = new Ramp(PxTransform(PxVec3(.0f, 40.f, -10.f)), PxVec3(100.f, 100.f, 100.f));
			testBox->Color(color_palette[4]);
			Add(testBox);*/

			
			golfBall = new Sphere(PxTransform(PxVec3(0.f, 2.f, -45.f)), golfBallRadius);
			//golfBall = new Sphere(PxTransform(PxVec3(-88.f, 2.f, 194.f)), golfBallRadius);
			golfBall->Name("Golf Ball");
			golfBall->Color(color_palette[4]);
			golfballRigid = (PxRigidDynamic*)golfBall->Get();
			golfballRigid->setMassSpaceInertiaTensor(PxVec3(0.f));
			golfballRigid->setMass(10);
			Add(golfBall);

			/*while (spawnCounter <= 2000)
			{
				golfBalls[spawnCounter] = new Sphere(PxTransform(PxVec3(golfSpawnX, golfSpawnY, -70.f)), golfBallRadius);
				golfBalls[spawnCounter]->Color(color_palette[6]);
				golfballRigid = (PxRigidDynamic*)golfBalls[spawnCounter]->Get();
				golfballRigid->setMassSpaceInertiaTensor(PxVec3(0.f));
				golfballRigid->setMass(10);
				Add(golfBalls[spawnCounter]);

				if (golfSpawnX == 10)
				{
					golfSpawnX = 0;
					golfSpawnY++;
				}
				
				golfSpawnX++;
				spawnCounter++;


			}*/


			PxQuat ramp1Rotation = PxQuat(-0.3, PxVec3(1.f, 0.f, 0.f));
			ramp1 = new Ramp(PxTransform(PxVec3(.0f, 3.f, 59.5f), ramp1Rotation), PxVec3(10.f, 10.f, 10.f));
			ramp1->Color(color_palette[3]);
			Add(ramp1);

			PxQuat ramp2Rotation = PxQuat(0.3, PxVec3(1.f, 0.f, 0.f));
			ramp2 = new Ramp(PxTransform(PxVec3(.0f, 2.5f, 85.0f), ramp2Rotation), PxVec3(10.f, 10.f, 10.f));
			ramp2->Color(color_palette[3]);
			Add(ramp2);

			windmillPlatform = new Platform(PxTransform(PxVec3(.0f, -1.f, 125.f)), PxVec3(10.f, 10.f, 60.0f));
			windmillPlatform->Color(color_palette[1]);
			Add(windmillPlatform);

		



			box = new Box(PxTransform(PxVec3(.0f, 6.f, 120.f)));
			box->Color(color_palette[0]);
			box->SetKinematic(1, 0);
			box2 = new Box(PxTransform(PxVec3(3.f, .5f, .0f)));
			RevoluteJoint tunnelJoint1(box, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), box2, PxTransform(PxVec3(0.f, 5.f, 0.f)));
			tunnelJoint1.DriveVelocity(3.f);
			Add(box);
			Add(box2);

			box3 = new Box(PxTransform(PxVec3(.0f, 6.f, 125.f)));
			box3->Color(color_palette[0]);
			box3->SetKinematic(1, 0);
			box4 = new Box(PxTransform(PxVec3(3.f, .5f, .0f)));
			RevoluteJoint tunnelJoint2(box3, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), box4, PxTransform(PxVec3(0.f, 5.f, 0.f)));
			tunnelJoint2.DriveVelocity(3.f);
			Add(box3);
			Add(box4);

			box5 = new Box(PxTransform(PxVec3(.0f, 6.f, 130.f)));
			box5->Color(color_palette[0]);
			box5->SetKinematic(1, 0);
			box6 = new Box(PxTransform(PxVec3(3.f, .5f, .0f)));
			RevoluteJoint tunnelJoint3(box5, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), box6, PxTransform(PxVec3(0.f, 5.f, 0.f)));
			tunnelJoint3.DriveVelocity(3.f);
			Add(box5);
			Add(box6);

			flagpole = new Box(PxTransform(PxVec3(-100.5f, 6.f, 194.f)), PxVec3(.25f, 7.f, .25f));
			flagpole->SetKinematic(1, 0);
			Add(flagpole);

			/*goalBox = new Box(PxTransform(PxVec3(-90.5f, -4.5f, 194.f)));
			goalBox->Color(color_palette[0]);
			goalBox->SetKinematic(1, 0);
			goalBox->SetTrigger(1);
			Add(goalBox);
*/
			slidingDoor = new Slider(PxTransform(PxVec3(0.f, 15.f, 150.f)), PxVec3(10.f, 10.f, 2.f));
			slidingDoor->SetKinematic(1, 1);
			slidingDoor->Color(color_palette[10]);
			Add(slidingDoor);

			PxQuat flagRot = PxQuat(-1.57f, PxVec3(0.f, 0.f, 1.f));
			cloth = new Cloth(PxTransform(PxVec3(-100.3f, 13.f, 194.f), flagRot), PxVec2(5.f, 2.5f), 30, 30);
			cloth->Color(color_palette[2]);
			((PxCloth*)cloth->Get())->setExternalAcceleration(PxVec3(15.0f, 0.f, 0.f));
			Add(cloth);
			//collision filter flags
			 box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1);
			 box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);
			 box2->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);



			
		}

		//Custom udpate function
		virtual void CustomUpdate()
		{
			/*if (debugCounter > 30)
			{
				float x, y, z;
				x = golfballRigid->getGlobalPose().p.x;
				y = golfballRigid->getGlobalPose().p.y;
				z = golfballRigid->getGlobalPose().p.z;
				cerr << "Golfball Position x: " + std::to_string(x) + " | y: " + std::to_string(y) + " | z: " + std::to_string(z) << endl;
				debugCounter = 0.0f;
			}*/
			PxRigidActor* wallActor = (PxRigidActor*)slidingDoor->Get();
			PxTransform wallPosition = wallActor->getGlobalPose();
			PxTransform ballPosition = golfballRigid->getGlobalPose();

			if (wallUp == true)
			{
				wallPosition.p.y = wallPosition.p.y + 0.5;
			}
			if (wallUp == false)
			{
				wallPosition.p.y = wallPosition.p.y - 0.5;
			}
			if (wallPosition.p.y > 20)
			{
				wallUp = false;
			}
			if (wallPosition.p.y < 5)
			{
				wallUp = true;
			}

			


			wallActor->setGlobalPose(wallPosition);
			/*debugCounter++;*/

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
