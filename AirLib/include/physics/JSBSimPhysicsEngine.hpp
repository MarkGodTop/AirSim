#ifndef airsim_core_JSBSimPhysicsEngine_hpp
#define airsim_core_JSBSimPhysicsEngine_hpp

#include "PhysicsBody.hpp"
#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "common/CommonStructs.hpp"
#include "FGFDMExec.h"
#include "physics/Kinematics.hpp"
#include "simgear/misc/sg_path.hxx"
#include <math.h>
#include "JSBSimAGLevelComponent.h"
#include "FDMTypes.h"
#include "PhysicsEngineBase.hpp"
#include "UEGroundCallback.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGAircraft.h"
#include "models/FGFCS.h"
#include "models/FGGroundReactions.h"
#include "models/FGInertial.h"
#include "models/FGLGear.h"
#include "models/FGMassBalance.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGPiston.h"
#include "models/propulsion/FGTurbine.h"
#include "models/propulsion/FGTurboProp.h"
// #include "Interfaces/IPluginManager.h"

namespace msr { namespace airlib {

class JSBSimPhysicsEngine : public UpdatableObject
{
	//interface for virtual functions to be implemented by a derived class
public: 
	virtual void setCollisionInfo(const CollisionInfo& collision_info)
	{
		collision_info_ = collision_info;
	}

	CollisionResponse& getCollisionResponseInfo()
	{
		return collision_response_;
	}
	
public: //methods
	JSBSimPhysicsEngine()
	{
		setName("JSBSimPhysicsEngine");
		setModelPath("");
		initialize();
	}

	JSBSimPhysicsEngine(UJSBSimAGLevelComponent* InJsbSimAGLevelComponent)
	{
		setName("JSBSimPhysicsEngine");
		setModelPath("");
		initializeByComponent(InJsbSimAGLevelComponent);
	}

	void initializeByComponent(UJSBSimAGLevelComponent* InJsbSimAGLevelComponent)
	{
				// Construct the JSBSim FDM
		Exec = new JSBSim::FGFDMExec();

		// Get pointer to main components
		Atmosphere = Exec->GetAtmosphere();
		Winds = Exec->GetWinds();
		FCS = Exec->GetFCS();
		MassBalance = Exec->GetMassBalance();
		Propulsion = Exec->GetPropulsion();

		//Start with Engine Running
		if(Propulsion)
			Propulsion->InitRunning(-1);
		
		Aircraft = Exec->GetAircraft();
		Propagate = Exec->GetPropagate();
		Auxiliary = Exec->GetAuxiliary();
		Inertial = Exec->GetInertial();
			Inertial->SetGroundCallback(new UEGroundCallback(InJsbSimAGLevelComponent));
		Aerodynamics = Exec->GetAerodynamics();
		GroundReactions = Exec->GetGroundReactions();
		Accelerations = Exec->GetAccelerations();
		IC = Exec->GetIC();
		PropertyManager = Exec->GetPropertyManager();
		
		loadJSBSimPaths(model_name_);
		
		//Initialize jsb ic by xml or double values
#if 1
		IC->SetAltitudeASLFtIC(3000.0f * METER_TO_FEET);
		IC->SetAltitudeAGLFtIC(3000.0f * METER_TO_FEET);
		Exec->GetPropagate()->SetInitialState(IC.get());
		//delete Fgic;
		
#else
		const SGPath ic_file("reset00.xml");
		if(IC->Load(ic_file))
		{
			UE_LOG(LogTemp,Warning,TEXT("Load IC File Succeed!!!"));
		}
#endif

		const uint32 EngineCount = Propulsion->GetNumEngines();
		
		if (EngineCount > 0)
		{
			for(uint32 i=0;i<EngineCount;i++)
			{
				// Allocate UE equivalent structures
				EngineCommands.Add(FEngineCommand());

				// UE_LOG(LogTemp,Warning,TEXT("** EngineCommands add command %d **"),i);
			}
		}
		
		Exec->Setdt(delta_t_);
		const bool success = Exec->RunIC(); // causes sim time to reset to 0.0, returns true if successful
		if (!success)
		{
			UE_LOG(LogTemp,Error,TEXT("ERROR:Exec Run IC failed"));
		}
	}
	
	void initialize()
	{
		// Construct the JSBSim FDM
		Exec = new JSBSim::FGFDMExec();

		// Get pointer to main components
		Atmosphere = Exec->GetAtmosphere();
		Winds = Exec->GetWinds();
		FCS = Exec->GetFCS();
		MassBalance = Exec->GetMassBalance();
		Propulsion = Exec->GetPropulsion();

		//Start with Engine Running
		if(Propulsion)
			Propulsion->InitRunning(-1);
		
		Aircraft = Exec->GetAircraft();
		Propagate = Exec->GetPropagate();
		Auxiliary = Exec->GetAuxiliary();
		Inertial = Exec->GetInertial();
			Inertial->SetGroundCallback(new UEGroundCallback());
		Aerodynamics = Exec->GetAerodynamics();
		GroundReactions = Exec->GetGroundReactions();
		Accelerations = Exec->GetAccelerations();
		IC = Exec->GetIC();
		PropertyManager = Exec->GetPropertyManager();
		
		loadJSBSimPaths(model_name_);
		// const SGPath ic_file("reset01");
		
		// check(IC);

		//Initialize jsb ic by xml or double values
#if 1
		// IC->SetLongitudeDegIC(40.0f); 
		// IC->SetGeodLatitudeDegIC(40.0f);
		IC->SetAltitudeASLFtIC(800.0f * METER_TO_FEET);
		IC->SetAltitudeAGLFtIC(800.0f * METER_TO_FEET);
		// IC->SetPhiDegIC(0.0f);
		// IC->SetPsiDegIC(0.0f + 90);
		// IC->SetThetaDegIC(0.0f);
		//
		// // Wind Speed
		// IC->SetWindDirDegIC(0.0f);
		// IC->SetWindMagKtsIC(0.0f);
		//
		// // Aircraft Speed
		// Fgic->SetVcalibratedKtsIC(InitialCalibratedAirSpeedKts);
		
		Exec->GetPropagate()->SetInitialState(IC.get());
		//delete Fgic;
		
#else
		const SGPath ic_file("reset00.xml");
		if(IC->Load(ic_file))
		{
			UE_LOG(LogTemp,Warning,TEXT("Load IC File Succeed!!!"));
		}
#endif

		const uint32 EngineCount = Propulsion->GetNumEngines();
		
		if (EngineCount > 0)
		{
			for(uint32 i=0;i<EngineCount;i++)
			{
				// Allocate UE equivalent structures
				EngineCommands.Add(FEngineCommand());

				// UE_LOG(LogTemp,Warning,TEXT("** EngineCommands add command %d **"),i);
			}
		}
		
		Exec->Setdt(delta_t_);
		const bool success = Exec->RunIC(); // causes sim time to reset to 0.0, returns true if successful
		if (!success)
		{
			UE_LOG(LogTemp,Error,TEXT("ERROR:Exec Run IC failed"));
		}
	}

	virtual void update() override
	{
		UpdatableObject::update();
		Exec->Setdt(delta_t_);
		CopyToJSBSim();
		if(Exec->Run())
		{
			updateKinematicState();
			updateControlState();
		}
		// if(GEngine)
		// {
		// 	GEngine->AddOnScreenDebugMessage(
		// 		2,
		// 		1.0f,
		// 		FColor::Purple,
		// 		FString::Printf(TEXT("控制状态:{ aileron:%f, elevator:%f, throttle:%f, rudder:%f }"),
		// 			jsbsim_control_state_[0],
		// 			jsbsim_control_state_[1],
		// 			jsbsim_control_state_[2],
		// 			jsbsim_control_state_[3]));
		// }
	}

	virtual void resetImplementation() override
	{
		const int no_output_reset_mode = 0;
	 Exec->ResetToInitialConditions(no_output_reset_mode); // multiple modes here not quite sure which is the best to set
	}

	void reportState(StateReporter& reporter) override
	{
		// call base implementation
		UpdatableObject::reportState(reporter);
	}

	// set the normalized control input, values are given between -1 and +1
	void setControlCommand(double aileron, double elevator, double throttle, double rudder)
	{
		setProperty("fcs/aileron-cmd-norm", aileron); // range -1 to +1
		setProperty("fcs/elevator-cmd-norm", elevator); // range -1 to +1
		setProperty("fcs/throttle-cmd-norm", throttle); // range 0 to +1 
		setProperty("fcs/rudder-cmd-norm", rudder); // range -1 ot +1 note x8 has no rudder
	}

	FVector GetEcefPosition() const
	{
		return FVector(position_ecef_.x(),position_ecef_.y(),position_ecef_.z());
	}
	
	void setModelPath(std::string model_name)
	{
		model_name_ = model_name;
		if(model_name == "") {
			model_name_ = "f16";
		}
	}

	void setControlCommands(const FFlightControlCommands& InCommands)
	{
		Commands = InCommands;
	}

	FFlightControlCommands getControlCommands() const
	{
		return Commands;
	}

	void setEngineCommands(const TArray<struct FEngineCommand> InEngineCommands)
	{
		EngineCommands = InEngineCommands;
	}

	TArray<struct FEngineCommand> getEngineCommands() const
	{
		return EngineCommands;
	}
	
	void setCollisionInfo(CollisionInfo& collision_info) const
	{
		collision_info = collision_info_;
	}

	Kinematics::State getKinematicState() const
	{
		return current_state_;
	}

	Pose& getPose()
	{
		return current_state_.pose;
	}

	vector<double>& getControlState()
	{
		return jsbsim_control_state_;
	}

	void setDeltaT(double delta_t)
	{
		delta_t_ = delta_t;
	}

	double getDeltaT() const
	{
		return delta_t_;
	}

	double getTime() const
	{
		return Exec->GetSimTime();
	}
	
	JSBSim::FGFDMExec getJSBSim()
	{
		return  *Exec;
	}

	//Copy control commands to jsbsim, Call in update()
	void CopyToJSBSim()
	{
		if(FCS==nullptr) return;
		
		// Basic flight controls
		FCS->SetDaCmd(Commands.Aileron);
		FCS->SetRollTrimCmd(Commands.RollTrim);
		FCS->SetDeCmd(Commands.Elevator);
		FCS->SetPitchTrimCmd(Commands.PitchTrim);
		FCS->SetDrCmd(-Commands.Rudder); // Rudder
		FCS->SetDsCmd(Commands.Rudder); // Steering
		FCS->SetYawTrimCmd(-Commands.YawTrim);
		FCS->SetDfCmd(Commands.Flap);
		FCS->SetDsbCmd(Commands.SpeedBrake);
		FCS->SetDspCmd(Commands.Spoiler);
		
		// Gears and Brake controls
		FCS->SetLBrake(FMath::Max(Commands.LeftBrake, Commands.ParkingBrake));
		FCS->SetRBrake(FMath::Max(Commands.RightBrake, Commands.ParkingBrake));
		FCS->SetCBrake(FMath::Max(Commands.CenterBrake, Commands.ParkingBrake));
		FCS->SetGearCmd(Commands.GearDown);

		// setProperty("fcs/aileron-cmd-norm", Commands.Aileron); // range -1 to +1
		// setProperty("fcs/elevator-cmd-norm", Commands.Elevator); // range -1 to +1
		// setProperty("fcs/rudder-cmd-norm", Commands.Rudder); // range -1 ot +1 note x8 has no rudder
		
		ApplyEnginesCommands(); 

		// UE_LOG(LogTemp,Warning,TEXT("** Rudder:%f **"),Commands.Rudder);
		// UE_LOG(LogTemp,Warning,TEXT("** Elevator:%f **"),Commands.Elevator);
		
		//TODO:
		// CopyTankPropertiesToJSBSim();
		// CopyGearPropertiesToJSBSim();

		FCS->Run(false);
	}
	
protected:
	void loadJSBSimPaths(std::string model_path)
	{
		//Get the base directory of this plugin
		
		const FString BaseDir = FPaths::Combine(FPaths::ProjectPluginsDir(),TEXT("JSBSimFlightDynamicsModel"));
		const FString RootDirRelative = FPaths::Combine(*BaseDir, TEXT("Resources/JSBSim"));
		const FString& RootDir = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*RootDirRelative);
		
		// Set data paths...
		const FString AircraftPath(TEXT("aircraft"));
		const FString EnginePath(TEXT("engine"));
		const FString SystemPath(TEXT("systems"));

		Exec->SetRootDir(SGPath(TCHAR_TO_UTF8(*RootDir)));
		Exec->SetAircraftPath(SGPath(TCHAR_TO_UTF8(*AircraftPath)));
		Exec->SetEnginePath(SGPath(TCHAR_TO_UTF8(*EnginePath)));
		Exec->SetSystemsPath(SGPath(TCHAR_TO_UTF8(*SystemPath)));
		
		if (Exec->LoadModel(model_path))
		{
			UE_LOG(LogTemp,Display,TEXT("Load Model Succeed"));
		}
		else
		{
			UE_LOG(LogTemp,Display,TEXT("Load Model Failed"));
		}
	}
	
	// get the the value of a property from JSBSim
	virtual double getProperty(const std::string& property_name)
	{
		const double property_value = Exec->GetPropertyValue(property_name);
		return property_value;
	}

	// set a property value
	virtual void setProperty(const std::string& property_name, double property_value)
	{
	 Exec->SetPropertyValue(property_name, property_value);
	}

	// get current jsbsim state
	virtual	void updateKinematicState()
	{
		getPositionMeters();
		current_state_.pose.position = position_;
		// UE_LOG(LogTemp,Log,TEXT("current_state.pose.position:(%f,%f,%f)"),current_state_.pose.position.x(),current_state_.pose.position.y(),current_state_.pose.position.z());
		// UE_LOG(LogTemp,Log,TEXT("position_:                  (%f,%f,%f)"),position_.x(),position_.y(),position_.z());
		getOrientationRadians();
		current_state_.pose.orientation = orientation_;
		getLinearVelocity();
		current_state_.twist.linear = linear_velocity_;
		getAngularVelocity();
		current_state_.twist.angular = angular_velocity_;
		getLinearAccel();
		current_state_.accelerations.linear = linear_acceleration_;
		getAngularAccel();
		current_state_.accelerations.angular = angular_acceleration_;
	}

	// gets the effective control deflection in radians
	virtual void updateControlState()
	{
		const float aileron = getProperty("fcs/effective-aileron-pos");
		const float elevator = getProperty("fcs/elevator-pos-rad");
		const float throttle = getProperty("propulsion/engine/thrust-lbs"); // pounds of thrust engine is producing
		const float rudder = getProperty("fcs/rudder-pos-norm");
		jsbsim_control_state_ = { aileron, elevator, throttle, rudder };
	}
	
	// get the distance from the earth zero point in metres
	void getPositionMeters()
	{
		const float lat_deg = getProperty("position/lat-geod-deg");
		const float long_deg = getProperty("position/long-gc-deg");
		const float altitude_m = getProperty("position/h-sl-ft") / 3.28;
		const float lat_m = 111320 * lat_deg;
		const float long_m = 40075000 * long_deg * cos(lat_deg * (pi / 180.0)) / 360.0;
		
		// UE_LOG(LogTemp,Warning,TEXT("** position(%f,%f,%f) **"),lat_m,long_m,-altitude_m);
		position_ = { lat_m, long_m, - altitude_m };
		position_ecef_ = { lat_deg,long_deg,altitude_m };
		
		// getLocalTransform();
		// const float lat_m = VRPLocalPosition.X/100.0f;
		// const float long_m = VRPLocalPosition.Y/100.0f;
		// const float altitude_m = VRPLocalPosition.Z/100.0f;
		// UE_LOG(LogTemp,Log,TEXT("** position(%f,%f,%f) **"),lat_m,long_m,-altitude_m);
		// position_ = { lat_m, long_m, - altitude_m };
	}

	// get the aircraft's local transforms
	void getLocalTransform()
	{
		if(MassBalance == nullptr || Aircraft == nullptr || GroundReactions == nullptr)
			return;

		// Structural Frame To Actor Frame
		FMatrix StructuralToActorMatrix(FMatrix::Identity);
		StructuralToActorMatrix.SetAxis(0, FVector(-1, 0, 0));
		StructuralToActorMatrix.SetAxis(1, FVector(0, 1, 0));
		StructuralToActorMatrix.SetAxis(2, FVector(0, 0, 1));
		StructuralToActorMatrix.SetOrigin(StructuralFrameOrigin);
		StructuralToActor.SetFromMatrix(StructuralToActorMatrix);

		// Get Gravity Center
		JSBSim::FGColumnVector3 CGLocationStructural = MassBalance->StructuralToBody(JSBSim::FGColumnVector3()) * FEET_TO_CENTIMETER;
		CGLocalPosition = StructuralToActor.TransformPosition(FVector(CGLocationStructural(1), CGLocationStructural(2), CGLocationStructural(3)));

		// Body Frame to Actor Frame
		FMatrix BodyToActorMatrix(FMatrix::Identity);
		BodyToActorMatrix.SetAxis(0, FVector(1, 0, 0));
		BodyToActorMatrix.SetAxis(1, FVector(0, 1, 0));
		BodyToActorMatrix.SetAxis(2, FVector(0, 0, -1));
		BodyToActorMatrix.SetOrigin(CGLocalPosition);
		BodyToActor.SetFromMatrix(BodyToActorMatrix);

		// Eye Position
		JSBSim::FGColumnVector3 EPLocationStructural = Aircraft->GetXYZep() * INCH_TO_CENTIMETER;
		EPLocalPosition = StructuralToActor.TransformPosition(FVector(EPLocationStructural(1), EPLocationStructural(2), EPLocationStructural(3)));

		// Visual Reference Position
		JSBSim::FGColumnVector3 VRPLocationStructural = Aircraft->GetXYZvrp() * INCH_TO_CENTIMETER;
		VRPLocalPosition = StructuralToActor.TransformPosition(FVector(VRPLocationStructural(1), VRPLocationStructural(2), VRPLocationStructural(3)));

		// Gear Locations
		for (int i = 0; i < GroundReactions->GetNumGearUnits(); i++)
		{
			std::shared_ptr<JSBSim::FGLGear> Gear = GroundReactions->GetGearUnit(i);
			if ((int32)i < Gears.Num())
			{
				JSBSim::FGColumnVector3 GearBodyLocation = Gear->GetBodyLocation() * FEET_TO_CENTIMETER;
				Gears[i].RelativeLocation = BodyToActor.TransformPosition(FVector(GearBodyLocation(1), GearBodyLocation(2), GearBodyLocation(3)));
			}
		}
	}
	
	// get the aircraft's orientation in radians
	void getOrientationRadians()
	{
		const float pitch = getProperty("attitude/pitch-rad");
		const float roll = getProperty("attitude/roll-rad");
		const float yaw = getProperty("attitude/psi-deg") * (pi / 180.0);
		Vector3r euler_orientation = { pitch, roll, yaw };
		// UE_LOG(LogTemp,Log,TEXT("** rotation(%f,%f,%f) **"),pitch,roll,yaw);
		orientation_ = VectorMath::toQuaternion(
			euler_orientation[0],
			euler_orientation[1],
			euler_orientation[2]);
	}

	void getLinearVelocity()
	{
		const float north_vel = getProperty("velocities/v-north-fps") / 3.28;
		const float east_vel = getProperty("velocities/v-east-fps") / 3.28;
		const float vertical_vel = getProperty("velocities/v-down-fps") / 3.28;
		linear_velocity_ = { north_vel, east_vel, -vertical_vel };
	}

	void getAngularVelocity()
	{
		const float p = getProperty("velocities/p-rad_sec");
		const float q = getProperty("velocities/q-rad_sec");
		const float r = getProperty("velocities/r-rad_sec");
		angular_velocity_ = { p, q, r };
	}

	void getLinearAccel()
	{
		const float ax = getProperty("accelerations/udot-ft_sec2") / 3.28;
		const float ay = getProperty("accelerations/vdot-ft_sec2") / 3.28;
		const float az = getProperty("accelerations/wdot-ft_sec2") / 3.28;
		linear_acceleration_ = { ax, ay, az };
	}

	void getAngularAccel()
	{
		const float pdot = getProperty("accelerations/pdot-rad_sec2");
		const float qdot = getProperty("accelerations/qdot-rad_sec2");
		const float rdot = getProperty("accelerations/rdot-rad_sec2");
		angular_acceleration_ = {pdot, qdot, rdot};
	}

	void ApplyEnginesCommands()
	{
		// Global to all engines
		//TODO:@fixme->现在默认无限燃料
		Propulsion->SetFuelFreeze(false);
		
		// For each engine
		const int32 EngineCount = EngineCommands.Num();

		// UE_LOG(LogTemp,Warning,TEXT("** ApplyEnginesCommands engine's number:%d **"),EngineCount);
		
		for (int32 i = 0; i < EngineCount; i++)
		{
			FEngineCommand EngineCommand = EngineCommands[i];
			
			// Global FCS Commands
			FCS->SetThrottleCmd(i, 1);
			FCS->SetMixtureCmd(i, EngineCommand.Mixture);
			FCS->SetPropAdvanceCmd(i, EngineCommand.PropellerAdvance);
			FCS->SetFeatherCmd(i, EngineCommand.PropellerFeather);

			// UE_LOG(LogTemp,Warning,TEXT("EngineCommand Throttle:%f"),FCS->GetThrottleCmd(0));
			
			// Common FGEngine code block
			std::shared_ptr < JSBSim::FGEngine> CommonEngine = Propulsion->GetEngine(i);
			CommonEngine->SetStarter(true);
			CommonEngine->SetRunning(true);

			switch (Propulsion->GetEngine(i)->GetType())
			{
			case JSBSim::FGEngine::etPiston:
				{
					// FGPiston code block
					std::shared_ptr < JSBSim::FGPiston> PistonEngine = std::static_pointer_cast<JSBSim::FGPiston>(Propulsion->GetEngine(i));
						PistonEngine->SetMagnetos((int)EngineCommand.Magnetos);
					break;
				}
			case JSBSim::FGEngine::etTurbine:
				{
					// FGTurbine code block
					std::shared_ptr < JSBSim::FGTurbine> TurbineEngine = std::static_pointer_cast<JSBSim::FGTurbine>(Propulsion->GetEngine(i));
						TurbineEngine->SetReverse(EngineCommand.Reverse);
						TurbineEngine->SetCutoff(EngineCommand.CutOff);
						TurbineEngine->SetIgnition(EngineCommand.Ignition);
						TurbineEngine->SetAugmentation(EngineCommand.Augmentation);
						TurbineEngine->SetInjection(EngineCommand.Injection);
						break;
				}
			case JSBSim::FGEngine::etRocket:
				{
					// FGRocket code block
					// FGRocket* RocketEngine = (FGRocket*)Propulsion->GetEngine(i);
					break;
				}
			case JSBSim::FGEngine::etTurboprop:
				{
					// FGTurboProp code block
					std::shared_ptr < JSBSim::FGTurboProp> TurboPropEngine = std::static_pointer_cast<JSBSim::FGTurboProp>(Propulsion->GetEngine(i));
					TurboPropEngine->SetReverse(EngineCommand.Reverse);
					TurboPropEngine->SetCutoff(EngineCommand.CutOff);
					TurboPropEngine->SetGeneratorPower(EngineCommand.GeneratorPower);
					TurboPropEngine->SetCondition(EngineCommand.Condition);
					break;
				}
			default:
				break;
			}
		}
	}
	
protected:
	CollisionInfo collision_info_;
	JSBSim::FGFDMExec* Exec = nullptr;

	std::shared_ptr<JSBSim::FGAtmosphere> Atmosphere = nullptr;
	std::shared_ptr<JSBSim::FGWinds> Winds = nullptr;
	std::shared_ptr<JSBSim::FGFCS> FCS = nullptr;
	std::shared_ptr<JSBSim::FGMassBalance> MassBalance = nullptr;
	std::shared_ptr<JSBSim::FGPropulsion> Propulsion = nullptr;

	std::shared_ptr<JSBSim::FGAircraft> Aircraft = nullptr;
	std::shared_ptr<JSBSim::FGPropagate> Propagate = nullptr;
	std::shared_ptr<JSBSim::FGAuxiliary> Auxiliary = nullptr;
	std::shared_ptr<JSBSim::FGInertial> Inertial = nullptr;
	std::shared_ptr<JSBSim::FGAerodynamics> Aerodynamics = nullptr;

	std::shared_ptr<JSBSim::FGGroundReactions> GroundReactions = nullptr;
	std::shared_ptr<JSBSim::FGAccelerations> Accelerations = nullptr;

	std::shared_ptr<JSBSim::FGPropertyManager> PropertyManager = nullptr;
	std::shared_ptr<JSBSim::FGInitialCondition> IC = nullptr;
	
	Kinematics::State current_state_;
	vector<double> jsbsim_control_state_ = { 0, 0, 0, 0 };
	Vector3r position_;
	Vector3r position_ecef_;
	Quaternionr orientation_;
	Vector3r linear_velocity_;
	Vector3r angular_velocity_;
	Vector3r linear_acceleration_;
	Vector3r angular_acceleration_;
	CollisionResponse collision_response_;
	std::string model_name_;
	double pi = 3.1415926535897932;
	double delta_t_ = 0.0021; // set the simulation update rate, defaults to 480Hz

	//Offset value for F16 model
	FVector StructuralFrameOrigin = FVector(-565.0f,0.0f,185.0f);
	
	//Center of Gravity Location in Actor local frame
	FVector CGLocalPosition;

	//JSBSim's Eye Position in Actor local frame
	FVector EPLocalPosition;

	//JSBSim's Visual Reference Point Position in Actor local frame
	FVector VRPLocalPosition;

	TArray<struct FGear> Gears;
	FTransform StructuralToActor;
	FTransform BodyToActor;

	FFlightControlCommands Commands;
	TArray<struct FEngineCommand> EngineCommands;
};
	
}
}
#endif