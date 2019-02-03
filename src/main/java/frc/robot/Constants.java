package frc.robot;

import frc.robot.lib.util.ConstantsBase;

/**
 * Attribution: adapted from FRC Team 254
 */

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    // singleton class
    private static Constants instance = null;

    public static Constants getInstance() {
        if (instance == null) {
            instance = new Constants();
        }
        return instance;
    }

    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;
    public static int kTalonTimeoutMs = 5; // ms
    public static int kTalonPidIdx = 0; // 0 for non-cascaded PIDs, 1 for cascaded PIDs

    public static double kNominalBatteryVoltage = 12.0;

    // Bumpers
    public static double kCenterToFrontBumper = 18.0; // position of front bumper with respect to robot center of
                                                      // rotation
    public static double kCenterToExtendedIntake = 18.0; // position of intake sweetspot when extended with respect to
                                                         // robot center of rotation
    public static double kCenterToRearBumper = 18.0; // position of rear bumper with respect to robot center of rotation
    public static double kCenterToSideBumper = 18.0; // position of side bumper with respect to robot center of rotation
    public static double kCenterToCornerBumper = Math
            .sqrt(kCenterToRearBumper * kCenterToRearBumper + kCenterToSideBumper * kCenterToSideBumper);

    // Vision constants
    public static double kCameraFrameRate = 90.0;		// frames per second
    public static double kCameraPoseX        = +9.00;	// camera location with respect to robot center of rotation, X axis is in direction of travel
    public static double kCameraPoseY        =     0;	// camera location with respect to robot center of rotation, Y axis is positive to the left
    public static double kCameraPoseZ        =   9.0;	// camera location with respect to floor, Z axis is positive with increasing elevation
    public static double kCameraPoseThetaRad =   0.0;	// camera angle with respect to robot heading, in radians
    public static double kCameraPitchRad     =     0;   // camera vertical angle with respect to level ground, in radians
    public static double kCameraDeadband = 0.0;
    
    public static double kVisionMaxVel    = 20.0; // inches/sec  		
    public static double kVisionMaxAccel  = 20.0; // inches/sec^2		
    public static double kTargetWidthInches = 10.25;
    public static double kTargetHeightInches = 14.00;
    public static double kCenterOfTargetHeightInches = 13.25;
    public static double kTargetDistanceThresholdFromBumperInches = 18;		// inches to stop from target, measured from front bumper
    public static double kTargetDistanceThresholdFromCameraInches = kCenterToFrontBumper - kCameraPoseX + kTargetDistanceThresholdFromBumperInches;
    
    public static double kVisionCompletionTolerance = 1.0; 
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches
    

    // math for limit switch
    public static double kHatchQuadEncoderUnitsPerRev = 4096;
    public static double kHatchQuadEncoderDegsPerRev = 360;
    public static double kHatchEncoderUnitsPerDegs = kHatchQuadEncoderUnitsPerRev/kHatchQuadEncoderDegsPerRev;
    // Shooter Constants
    public static double kShooterPoseX        =     0;	// shooter location with respect to robot center of rotation, X axis is in direction of travel
    public static double kShooterPoseY        =     0;	// shooter location with respect to robot center of rotation, Y axis is positive to the left
    public static double kShooterPoseZ        =     0;	// shooter location with respect to floor, Z axis is positive with increasing elevation
    public static double kShooterPoseThetaRad =     0;	// shooter angle with respect to robot heading, in radians
    public static double kAutoAimPredictionTime =   0;	// set to 0 since we don't have a turret and need to point the entire robot


    // Motor Controllers
    // (Note that if multiple Talons are dedicated to a mechanism, any sensors are attached to the master)
    public static int kLeftMotorMasterTalonId = 1;
    public static int kLeftMotorSlave1TalonId = 2;
    public static int kRightMotorMasterTalonId = 3;
    public static int kRightMotorSlave1TalonId = 4;

    // motors inversions
    public static boolean kLeftMotorInverted = false;
    public static boolean kRightMotorInverted = true;
    public static boolean kLeftMotorSensorPhase = false;
    public static boolean kRightMotorSensorPhase = false;

	public static int kDriveTrainCurrentLimit = 25;

    // Wheel Encoder
    public static int    kQuadEncoderCodesPerRev = 256;
    public static int    kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static double kQuadEncoderStatusFramePeriod = 0.100;	// 100ms
    public static double kDriveSecondsFromNeutralToFull = 0.375;
    
    // CONTROL LOOP GAINS
    public static double kFullThrottleRPM = 520;	// measured max RPM using NI web interface
    public static double kFullThrottleEncoderPulsePer100ms = kFullThrottleRPM / 60.0 * kQuadEncoderStatusFramePeriod * kQuadEncoderUnitsPerRev; 
    

    // Joystick Controls
    public static int kXboxButtonA = 1;
    public static int kXboxButtonB = 2;
    public static int kXboxButtonX = 3;
    public static int kXboxButtonY = 4;
    public static int kXboxButtonLB = 5;
    public static int kXboxButtonRB = 6;

    public static int kXboxLStickXAxis = 0;
    public static int kXboxLStickYAxis = 1;
    public static int kXboxLTriggerAxis = 2;
    public static int kXboxRTriggerAxis = 3;
    public static int kXboxRStickXAxis = 4;
    public static int kXboxRStickYAxis = 5;

  //  Button Board Controls
	public static int kBumperButton 	= 1;
	public static int kRocketButton 	= 2;
    public static int kDefenseButton 	= 3;
    

    public static int kIntakeButton =           kXboxButtonRB;
    public static int kOuttakeButton =          kXboxButtonLB;
    public static int kGroundPickupButton =     kXboxButtonY;
    public static int kHatchShootButton =       kXboxButtonA;
    public static int kQuickTurnButton =        99;
    public static int kControlsReverseButton =  kXboxButtonB;
    public static int kVisionAssistanceButton = kXboxButtonX;
    


    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    //public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.BNO055;
    // public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.NAVX;
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;

};
