package frc.robot;

import frc.robot.lib.util.ConstantsBase;


/**
 * Attribution: adapted from FRC Team 254
 */


/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase
{
 	// singleton class
     private static Constants instance = null;
     public static Constants getInstance() 
     { 
         if (instance == null) {
             instance = new Constants();
         }
         return instance;
     }
     
    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;    
    public static int kTalonTimeoutMs = 5;	// ms
    public static int kTalonPidIdx = 0;		// 0 for non-cascaded PIDs, 1 for cascaded PIDs
    	
    public static double kNominalBatteryVoltage = 12.0;
    
    
    // Bumpers
    public static double kCenterToFrontBumper 		= 18.0;	// position of front bumper with respect to robot center of rotation
    public static double kCenterToExtendedIntake 	= 18.0;	// position of intake sweetspot when extended with respect to robot center of rotation
    public static double kCenterToRearBumper 		= 18.0;	// position of rear bumper with respect to robot center of rotation
    public static double kCenterToSideBumper 		= 18.0;	// position of side bumper with respect to robot center of rotation
	public static double kCenterToCornerBumper 		= Math.sqrt(kCenterToRearBumper*kCenterToRearBumper + kCenterToSideBumper*kCenterToSideBumper);


    
    
    // Vision constants
    public static double kCameraPoseX     = 0.0;	// camera location with respect to robot center of rotation, +X axis is in direction of travel
    public static double kCameraPoseY     = 0.0;	// camera location with respect to robot center of rotation, +Y axis is positive to the left
    public static double kCameraPoseTheta = 0.0;	// camera angle with respect to robot heading
    
    public static double kVisionMaxVel    = 20.0; // inches/sec  		
    public static double kVisionMaxAccel  = 20.0; // inches/sec^2		
    public static double kTargetWidthInches = 10.25;
    public static double kPegTargetDistanceThresholdFromBumperInches = 18;		// inches to stop from target, measured from front bumper
    public static double kPegTargetDistanceThresholdFromCameraInches = kCenterToFrontBumper - kCameraPoseX + kPegTargetDistanceThresholdFromBumperInches;
    public static double kVisionCompletionTolerance = 1.0; 
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches
    public static double kCameraFOVDegrees = 42.5;			// Camera Field of View (degrees)
    public static double kCameraHalfFOVRadians = kCameraFOVDegrees/2.0 * Math.PI/180.0;			// Half of Camera Field of View (radians)
    public static double kTangentCameraHalfFOV = Math.tan(kCameraHalfFOVRadians);
    public static double kCameraLatencySeconds = 0.240;			// Camera image capturing latency
    public static double kTargetLocationFilterConstant = (30.0 * kLoopDt);		// 30 time constants in 1 second
    

     // Motor Controllers
    // (Note that if multiple Talons are dedicated to a mechanism, any sensors are attached to the master)
	public static int kLeftMotorMasterTalonId   = 1;
	public static int kLeftMotorSlave1TalonId 	= 2;
    public static int kRightMotorMasterTalonId 	= 4;
	public static int kRightMotorSlave1TalonId 	= 5;

    // motors inversions
    public static boolean	kLeftMotorInverted  	= false;
    public static boolean	kRightMotorInverted 	= true;
    public static boolean	kLeftMotorSensorPhase 	= false;
    public static boolean	kRightMotorSensorPhase 	= false;

	public static int kDriveTrainCurrentLimit = 25;
	

    // Joystick Controls
    public static int kXboxButtonA  = 1;
    public static int kXboxButtonB  = 2;
    public static int kXboxButtonX  = 3;
    public static int kXboxButtonY  = 4;
    public static int kXboxButtonLB = 5;
    public static int kXboxButtonRB = 6;
    
    public static int kXboxLStickXAxis  = 0;
    public static int kXboxLStickYAxis  = 1;
    public static int kXboxLTriggerAxis = 2;
    public static int kXboxRTriggerAxis = 3;
    public static int kXboxRStickXAxis  = 4;
    public static int kXboxRStickYAxis  = 5;

	public static int kIntakeButton 			= kXboxButtonRB;
	public static int kOuttakeButton 			= kXboxButtonLB;
	public static int kQuickTurnButton 			= kXboxButtonX;

	
    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX; }
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.BNO055;

    
 };
   
  