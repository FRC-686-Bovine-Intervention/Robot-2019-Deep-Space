package frc.robot.auto.modes;

import frc.robot.Constants;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
    public static boolean rightSide = false;

    public enum TargetPositionEnum
    {
        CARGO_FRONT, CARGO_SIDE1, CARGO_SIDE2, CARGO_SIDE3, ROCKET_NEAR, ROCKET_FAR;
    }

	// dimensions of field components
	public static double kFieldLengthX = 648;       // 54'
	public static double kFieldLengthY = 324;       // 27'
    
    // Habitat
    public static double kHabWidthY = 128.0;        // not counting ramp
    public static double kHab3DepthX = 48.0;

    // Robot starting poses
	public static Pose centerLeftStartPose  = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, 0,                                            0);
	public static Pose centerRightStartPose = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, 0,                                            0);	
	public static Pose leftStartPose        = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, +kHabWidthY/2 - Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	public static Pose rightStartPose       = new Pose(kHab3DepthX + Constants.kCenterToFrontBumper, -kHabWidthY/2 + Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	public static Pose leftHab2StartPose    = new Pose(Constants.kCenterToFrontBumper, +kHabWidthY/2 - Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	public static Pose rightHab2StartPose   = new Pose(Constants.kCenterToFrontBumper, -kHabWidthY/2 + Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB

    // Rocket
    public static Vector2d kRocketCenter = new Vector2d(229.1, 162.0);          // TODO: fix this -- right now Y is too high
    public static double   kRocketCenterToHatch = 17.0;                         // distance from rocket center to center of hatches on diagonal
    public static double   kRocketAngleRad = 30.0*Vector2d.degreesToRadians ;   // angle from center to hatch, relative to back
    public static double   kRocketVisionDist = 40.0;                            // distance from which to turn on cameras
    public static double   kRocketTurnDist   = 30.0;                            // distance from which to turn towards rocket
    
    public static double   kNearRocketAngleRad = Math.PI - kRocketAngleRad;
    public static double   kFarRocketAngleRad =      0.0 - kRocketAngleRad;

    // Near Rocket
    public static Vector2d kNearRocketHatchPosition  = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketCenterToHatch, kNearRocketAngleRad) );
    public static Vector2d kNearRocketVisionPosition = kNearRocketHatchPosition.add(  Vector2d.magnitudeAngle(kRocketVisionDist, kNearRocketAngleRad) );
    public static Vector2d kNearRocketTurnPosition   = kNearRocketVisionPosition.add( Vector2d.magnitudeAngle(kRocketTurnDist,   kNearRocketAngleRad) );
 
    public static double   kNearRocketBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    public static double   kNearRocketBackupAngle1  = kNearRocketAngleRad; 
    public static double   kNearRocketBackupDist2   = 72.0;                 // backup2: fast backup 6 ft towards center of field
    public static double   kNearRocketBackupAngle2  = -Math.PI/2;                           
    public static double   kNearRocketBackupDist3   = 1.0;                  // backup3: just to keep robot pointed in right direction
    public static double   kNearRocketBackupAngle3  = 0.0;                           

    public static Vector2d kNearRocketBackupPosition1 = kNearRocketHatchPosition.add(   Vector2d.magnitudeAngle(kNearRocketBackupDist1, kNearRocketBackupAngle1));
    public static Vector2d kNearRocketBackupPosition2 = kNearRocketBackupPosition1.add( Vector2d.magnitudeAngle(kNearRocketBackupDist2, kNearRocketBackupAngle2));
    public static Vector2d kNearRocketBackupPosition3 = kNearRocketBackupPosition2.add( Vector2d.magnitudeAngle(kNearRocketBackupDist3, kNearRocketBackupAngle3));

    // Far Rocket
    public static Vector2d kFarRocketHatchPosition  = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketCenterToHatch, kFarRocketAngleRad) );
    public static Vector2d kFarRocketVisionPosition = kFarRocketHatchPosition.add(  Vector2d.magnitudeAngle(kRocketVisionDist, kFarRocketAngleRad) );
    public static Vector2d kFarRocketTurnPosition   = kFarRocketVisionPosition.add( Vector2d.magnitudeAngle(kRocketTurnDist,   kFarRocketAngleRad) );
 
    public static double   kFarRocketBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    public static double   kFarRocketBackupAngle1  = kFarRocketAngleRad; 
    public static double   kFarRocketBackupDist2   = 72.0;                 // backup2: fast backup 6 ft towards center of field
    public static double   kFarRocketBackupAngle2  = -Math.PI/2;                           
    public static double   kFarRocketBackupDist3   = 1.0;                  // backup3: just to keep robot pointed in right direction
    public static double   kFarRocketBackupAngle3  = 0.0;                           

    public static Vector2d kFarRocketBackupPosition1 = kFarRocketHatchPosition.add(   Vector2d.magnitudeAngle(kFarRocketBackupDist1, kFarRocketBackupAngle1));
    public static Vector2d kFarRocketBackupPosition2 = kFarRocketBackupPosition1.add( Vector2d.magnitudeAngle(kFarRocketBackupDist2, kFarRocketBackupAngle2));
    public static Vector2d kFarRocketBackupPosition3 = kFarRocketBackupPosition2.add( Vector2d.magnitudeAngle(kFarRocketBackupDist3, kFarRocketBackupAngle3));





    // Cargo Ship
    public static double   kCargoVisionDist = 30.0;                      // distance from which to turn turn on cameras (desired target should be mostly centered)
    public static double   kCargoTurnDist   = 40.0;                      // distance from which to turn towards cargo bay

    // Cargo Ship Front Bay
    public static Vector2d kCargoFrontHatchPosition = new Vector2d(220.3, 10.9);
    public static double   kCargoFrontAngleRad = -Math.PI;

    public static Vector2d kCargoFrontVisionPosition  = kCargoFrontHatchPosition.add( Vector2d.magnitudeAngle(kCargoVisionDist, kCargoFrontAngleRad) );
    public static Vector2d kCargoFrontTurnPosition    = kCargoFrontVisionPosition.add( Vector2d.magnitudeAngle(kCargoTurnDist,  kCargoFrontAngleRad) );
    
    public static double   kCargoFrontBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    public static double   kCargoFrontBackupAngle1  = kCargoFrontAngleRad; 
    public static double   kCargoFrontBackupDist2   = 72.0;                 // backup2: fast backup 6 ft towards center of field
    public static double   kCargoFrontBackupAngle2  = +Math.PI/2;                           
    public static double   kCargoFrontBackupDist3   = 1.0;                  // backup3: just to keep robot pointed in right direction
    public static double   kCargoFrontBackupAngle3  = 0.0;                           
    
    public static Vector2d kCargoFrontBackupPosition1 = kCargoFrontHatchPosition.add(   Vector2d.magnitudeAngle(kFarRocketBackupDist1, kFarRocketBackupAngle1));
    public static Vector2d kCargoFrontBackupPosition2 = kCargoFrontBackupPosition1.add( Vector2d.magnitudeAngle(kFarRocketBackupDist2, kFarRocketBackupAngle2));
    public static Vector2d kCargoFrontBackupPosition3 = kCargoFrontBackupPosition2.add( Vector2d.magnitudeAngle(kFarRocketBackupDist3, kFarRocketBackupAngle3));
    

    // Cargo Ship Side Bays
    public static Vector2d kCargoSide1HatchPosition = new Vector2d(260.8, 27.9);
    public static Vector2d kCargoSide2HatchPosition = new Vector2d(282.6, 27.9);
    public static Vector2d kCargoSide3HatchPosition = new Vector2d(304.3, 27.9);
    public static double   kCargoSideAngleRad = +Math.PI/2;

    public static Vector2d kCargoSide1VisionPosition = kCargoSide1HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist, kCargoSideAngleRad) );
    public static Vector2d kCargoSide2VisionPosition = kCargoSide2HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist, kCargoSideAngleRad) );
    public static Vector2d kCargoSide3VisionPosition = kCargoSide3HatchPosition.add(  Vector2d.magnitudeAngle(kCargoVisionDist, kCargoSideAngleRad) );

    public static Vector2d kCargoSide1TurnPosition   = kCargoSide1VisionPosition.add( Vector2d.magnitudeAngle(kCargoTurnDist,   kCargoSideAngleRad) );
    public static Vector2d kCargoSide2TurnPosition   = kCargoSide2VisionPosition.add( Vector2d.magnitudeAngle(kCargoTurnDist,   kCargoSideAngleRad) );
    public static Vector2d kCargoSide3TurnPosition   = kCargoSide3VisionPosition.add( Vector2d.magnitudeAngle(kCargoTurnDist,   kCargoSideAngleRad) );

    public static double   kCargoSideBackupDist1   = 6.0;                  // backup1: just back up a little away from hatch
    public static double   kCargoSideBackupAngle1  = kCargoSideAngleRad; 
    public static double   kCargoSideBackupDist2   = 36.0;                 // backup2: fast backup 3 ft towards center of field
    public static double   kCargoSideBackupAngle2  = kCargoSideAngleRad;                           
    public static double   kCargoSideBackupDist3   = 36.0;                 // backup3: point towards human station
    public static double   kCargoSideBackupAngle3  = 0.0;                           
    
    public static Vector2d kCargoSide1BackupPosition1 = kCargoSide1HatchPosition.add(  Vector2d.magnitudeAngle(kCargoSideBackupDist1,   kCargoSideBackupAngle1) );
    public static Vector2d kCargoSide2BackupPosition1 = kCargoSide2HatchPosition.add(  Vector2d.magnitudeAngle(kCargoSideBackupDist1,   kCargoSideBackupAngle1) );
    public static Vector2d kCargoSide3BackupPosition1 = kCargoSide3HatchPosition.add(  Vector2d.magnitudeAngle(kCargoSideBackupDist1,   kCargoSideBackupAngle1) );
    
    public static Vector2d kCargoSide1BackupPosition2 = kCargoSide1BackupPosition1.add(Vector2d.magnitudeAngle(kCargoSideBackupDist2,   kCargoSideBackupAngle2) );
    public static Vector2d kCargoSide2BackupPosition2 = kCargoSide2BackupPosition1.add(Vector2d.magnitudeAngle(kCargoSideBackupDist2,   kCargoSideBackupAngle2) );
    public static Vector2d kCargoSide3BackupPosition2 = kCargoSide3BackupPosition1.add(Vector2d.magnitudeAngle(kCargoSideBackupDist2,   kCargoSideBackupAngle2) );

    public static Vector2d kCargoSide1BackupPosition3 = kCargoSide1BackupPosition2.add(Vector2d.magnitudeAngle(kCargoSideBackupDist3,   kCargoSideBackupAngle3) );
    public static Vector2d kCargoSide2BackupPosition3 = kCargoSide2BackupPosition2.add(Vector2d.magnitudeAngle(kCargoSideBackupDist3,   kCargoSideBackupAngle3) );
    public static Vector2d kCargoSide3BackupPosition3 = kCargoSide3BackupPosition2.add(Vector2d.magnitudeAngle(kCargoSideBackupDist3,   kCargoSideBackupAngle3) );

    


    // Human Station
    public static Vector2d kHumanStationHatchPosition = new Vector2d(0.0, 136.3);
    public static double   kHumanStationAngleRad = 0.0;

    public static double   kHumanStationVisionDist = 96.0;                      // distance from which to turn turn on cameras (desired target should be mostly centered)
    public static double   kHumanStationTurnDist   = 30.0;                      // distance from which to turn towards human station
    
    public static Vector2d kHumanStationFarRocketMidPosition  = new Vector2d(200, 110);     // position to pass through when going FarRocket<-->HumanStation
    public static Vector2d kHumanStationSideCargoMidPosition  = new Vector2d(200, 110);     // position to pass through when going CargoSide<-->HumanStation
    public static Vector2d kHumanStationFrontCargoMidPosition = new Vector2d(130, 100);     // position to pass through when going HumanStation-->CargoFront

    public static Vector2d kHumanStationVisionPosition = kHumanStationHatchPosition.add(  Vector2d.magnitudeAngle(kHumanStationVisionDist, kHumanStationAngleRad) );
    public static Vector2d kHumanStationTurnPosition =   kHumanStationVisionPosition.add( Vector2d.magnitudeAngle(kHumanStationTurnDist,   kHumanStationAngleRad-20*Math.PI/180) );



    
    // gets

    // rightSide is set to true if the chosen start pose is on the right
	public static Pose getHab1CenterLeftStartPose()     { rightSide = false;   return centerLeftStartPose; }	
	public static Pose getHab1CenterRightStartPose()    { rightSide =  true;   return centerRightStartPose; }	
	public static Pose getHab1LeftStartPose()           { rightSide = false;   return leftStartPose; }  
    public static Pose getHab1RightStartPose()          { rightSide =  true;   return rightStartPose; } 
	public static Pose getHab2LeftStartPose()           { rightSide = false;   return leftHab2StartPose; }  
    public static Pose getHab2RightStartPose()          { rightSide =  true;   return rightHab2StartPose; } 
    
    // the y-coordinate is negated if the position is on the right side of the field (using complex conjugate)
	public static Vector2d getNearRocketHatchPosition()    { return (!rightSide ? kNearRocketHatchPosition  : kNearRocketHatchPosition.conj()); }	
	public static Vector2d getNearRocketTurnPosition()     { return (!rightSide ? kNearRocketTurnPosition   : kNearRocketTurnPosition.conj()); }	
	public static Vector2d getNearRocketVisionPosition()   { return (!rightSide ? kNearRocketVisionPosition : kNearRocketVisionPosition.conj()); }	
    public static Vector2d getNearRocketBackupPosition1()  { return (!rightSide ? kNearRocketBackupPosition1 : kNearRocketBackupPosition1.conj()); }	
    public static Vector2d getNearRocketBackupPosition2()  { return (!rightSide ? kNearRocketBackupPosition2 : kNearRocketBackupPosition2.conj()); }	
    public static Vector2d getNearRocketBackupPosition3()  { return (!rightSide ? kNearRocketBackupPosition3 : kNearRocketBackupPosition3.conj()); }	
    
	public static Vector2d getFarRocketHatchPosition()     { return (!rightSide ? kFarRocketHatchPosition   : kFarRocketHatchPosition.conj()); }	
	public static Vector2d getFarRocketTurnPosition()      { return (!rightSide ? kFarRocketTurnPosition    : kFarRocketTurnPosition.conj()); }	
	public static Vector2d getFarRocketVisionPosition()    { return (!rightSide ? kFarRocketVisionPosition  : kFarRocketVisionPosition.conj()); }	
	public static Vector2d getFarRocketBackupPosition1()   { return (!rightSide ? kFarRocketBackupPosition1  : kFarRocketBackupPosition1.conj()); }	
	public static Vector2d getFarRocketBackupPosition2()   { return (!rightSide ? kFarRocketBackupPosition2  : kFarRocketBackupPosition2.conj()); }	
	public static Vector2d getFarRocketBackupPosition3()   { return (!rightSide ? kFarRocketBackupPosition3  : kFarRocketBackupPosition3.conj()); }	

    public static Vector2d getCargoFrontHatchPosition()   { return (!rightSide ? kCargoFrontHatchPosition  : kCargoFrontHatchPosition.conj()); }	
    public static Vector2d getCargoFrontTurnPosition()    { return (!rightSide ? kCargoFrontTurnPosition   : kCargoFrontTurnPosition.conj()); }	
    public static Vector2d getCargoFrontVisionPosition()  { return (!rightSide ? kCargoFrontVisionPosition : kCargoFrontVisionPosition.conj()); }	
    public static Vector2d getCargoFrontBackupPosition1() { return (!rightSide ? kCargoFrontBackupPosition1 : kCargoFrontBackupPosition1.conj()); }	
    public static Vector2d getCargoFrontBackupPosition2() { return (!rightSide ? kCargoFrontBackupPosition2 : kCargoFrontBackupPosition2.conj()); }	
    public static Vector2d getCargoFrontBackupPosition3() { return (!rightSide ? kCargoFrontBackupPosition3 : kCargoFrontBackupPosition3.conj()); }	
   
    public static Vector2d getCargoSide1HatchPosition()   { return (!rightSide ? kCargoSide1HatchPosition   : kCargoSide1HatchPosition.conj()); }	
    public static Vector2d getCargoSide1TurnPosition()    { return (!rightSide ? kCargoSide1TurnPosition   : kCargoSide1TurnPosition.conj()); }	
    public static Vector2d getCargoSide1VisionPosition()  { return (!rightSide ? kCargoSide1VisionPosition : kCargoSide1VisionPosition.conj()); }	
    public static Vector2d getCargoSide1BackupPosition1() { return (!rightSide ? kCargoSide1BackupPosition1 : kCargoSide1BackupPosition1.conj()); }	
    public static Vector2d getCargoSide1BackupPosition2() { return (!rightSide ? kCargoSide1BackupPosition2 : kCargoSide1BackupPosition2.conj()); }	
    public static Vector2d getCargoSide1BackupPosition3() { return (!rightSide ? kCargoSide1BackupPosition3 : kCargoSide1BackupPosition3.conj()); }	
    
    public static Vector2d getCargoSide2HatchPosition()   { return (!rightSide ? kCargoSide2HatchPosition   : kCargoSide2HatchPosition.conj()); }	
    public static Vector2d getCargoSide2TurnPosition()    { return (!rightSide ? kCargoSide2TurnPosition   : kCargoSide2TurnPosition.conj()); }	
    public static Vector2d getCargoSide2VisionPosition()  { return (!rightSide ? kCargoSide2VisionPosition : kCargoSide2VisionPosition.conj()); }	
    public static Vector2d getCargoSide2BackupPosition1() { return (!rightSide ? kCargoSide2BackupPosition1 : kCargoSide2BackupPosition1.conj()); }	
    public static Vector2d getCargoSide2BackupPosition2() { return (!rightSide ? kCargoSide2BackupPosition2 : kCargoSide2BackupPosition2.conj()); }	
    public static Vector2d getCargoSide2BackupPosition3() { return (!rightSide ? kCargoSide2BackupPosition3 : kCargoSide2BackupPosition3.conj()); }	
    
    public static Vector2d getCargoSide3HatchPosition()   { return (!rightSide ? kCargoSide3HatchPosition   : kCargoSide3HatchPosition.conj()); }	
    public static Vector2d getCargoSide3TurnPosition()    { return (!rightSide ? kCargoSide3TurnPosition   : kCargoSide3TurnPosition.conj()); }	
    public static Vector2d getCargoSide3VisionPosition()  { return (!rightSide ? kCargoSide3VisionPosition : kCargoSide3VisionPosition.conj()); }	
    public static Vector2d getCargoSide3BackupPosition1() { return (!rightSide ? kCargoSide3BackupPosition1 : kCargoSide3BackupPosition1.conj()); }	
    public static Vector2d getCargoSide3BackupPosition2() { return (!rightSide ? kCargoSide3BackupPosition2 : kCargoSide3BackupPosition2.conj()); }	
    public static Vector2d getCargoSide3BackupPosition3() { return (!rightSide ? kCargoSide3BackupPosition3 : kCargoSide3BackupPosition3.conj()); }	
    
    public static Vector2d getHumanStationHatchPosition()  { return (!rightSide ? kHumanStationHatchPosition : kHumanStationHatchPosition.conj()); }
    public static Vector2d getHumanStationVisionPosition() { return (!rightSide ? kHumanStationVisionPosition : kHumanStationVisionPosition.conj()); }

    public static Vector2d getHumanStationFarRocketMidPosition()  = { return (!rightSide ? kHumanStationFarRocketMidPosition  : kHumanStationFarRocketMidPosition.conj()); }
    public static Vector2d getHumanStationSideCargoMidPosition()  = { return (!rightSide ? kHumanStationSideCargoMidPosition  : kHumanStationSideCargoMidPosition.conj()); }
    public static Vector2d getHumanStationFrontCargoMidPosition() = { return (!rightSide ? kHumanStationFrontCargoMidPosition : kHumanStationFrontCargoMidPosition.conj()); }


    public static Vector2d getTargetBackupTurnPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
    //    switch (_target)
    //     {
    //         case CARGO_FRONT:   rv =  getCargoFrontBackupTurnPosition(); break;
    //         case CARGO_SIDE1:   rv =  getCargoSide1BackupTurnPosition(); break;
    //         case CARGO_SIDE2:   rv =  getCargoSide2BackupTurnPosition(); break;
    //         case CARGO_SIDE3:   rv =  getCargoSide3BackupTurnPosition(); break;
    //         case ROCKET_NEAR:   rv =  getNearRocketBackupTurnPosition(); break;
    //         case ROCKET_FAR:    rv =  getFarRocketBackupTurnPosition(); break;
    //     }

        return rv;
    }

    public static Vector2d getTargetTurnPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
       switch (_target)
        {
            case CARGO_FRONT:   rv =  getCargoFrontTurnPosition(); break;
            case CARGO_SIDE1:   rv =  getCargoSide1TurnPosition(); break;
            case CARGO_SIDE2:   rv =  getCargoSide2TurnPosition(); break;
            case CARGO_SIDE3:   rv =  getCargoSide3TurnPosition(); break;
            case ROCKET_NEAR:   rv =  getNearRocketTurnPosition(); break;
            case ROCKET_FAR:    rv =  getFarRocketTurnPosition(); break;
        }

        return rv;
    }

    public static Vector2d getTargetVisionPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv =  getCargoFrontVisionPosition(); break;
            case CARGO_SIDE1:   rv =  getCargoSide1VisionPosition(); break;
            case CARGO_SIDE2:   rv =  getCargoSide2VisionPosition(); break;
            case CARGO_SIDE3:   rv =  getCargoSide3VisionPosition(); break;
            case ROCKET_NEAR:   rv =  getNearRocketVisionPosition(); break;
            case ROCKET_FAR:    rv =  getFarRocketVisionPosition(); break;
        }

        return rv;
    }

    public static Vector2d getTargetHatchPosition(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();

        switch (_target)
        {
            case CARGO_FRONT:   rv =  getCargoFrontHatchPosition(); break;
            case CARGO_SIDE1:   rv =  getCargoSide1HatchPosition(); break;
            case CARGO_SIDE2:   rv =  getCargoSide2HatchPosition(); break;
            case CARGO_SIDE3:   rv =  getCargoSide3HatchPosition(); break;
            case ROCKET_NEAR:   rv =  getNearRocketHatchPosition(); break;
            case ROCKET_FAR:    rv =  getFarRocketHatchPosition(); break;
        }

        return rv;
    }

    public static Vector2d getTargetBackupPosition1(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv =  getCargoFrontBackupPosition1(); break;
            case CARGO_SIDE1:   rv =  getCargoSide1BackupPosition1(); break;
            case CARGO_SIDE2:   rv =  getCargoSide2BackupPosition1(); break;
            case CARGO_SIDE3:   rv =  getCargoSide3BackupPosition1(); break;
            case ROCKET_NEAR:   rv =  getNearRocketBackupPosition1(); break;
            case ROCKET_FAR:    rv =  getFarRocketBackupPosition1(); break;
        }

        return rv;
    }

    public static Vector2d getTargetBackupPosition2(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv =  getCargoFrontBackupPosition2(); break;
            case CARGO_SIDE1:   rv =  getCargoSide1BackupPosition2(); break;
            case CARGO_SIDE2:   rv =  getCargoSide2BackupPosition2(); break;
            case CARGO_SIDE3:   rv =  getCargoSide3BackupPosition2(); break;
            case ROCKET_NEAR:   rv =  getNearRocketBackupPosition2(); break;
            case ROCKET_FAR:    rv =  getFarRocketBackupPosition2(); break;
        }

        return rv;
    }

    public static Vector2d getTargetBackupPosition3(TargetPositionEnum _target)
    {
        Vector2d rv = new Vector2d();
        
        switch (_target)
        {
            case CARGO_FRONT:   rv =  getCargoFrontBackupPosition3(); break;
            case CARGO_SIDE1:   rv =  getCargoSide1BackupPosition3(); break;
            case CARGO_SIDE2:   rv =  getCargoSide2BackupPosition3(); break;
            case CARGO_SIDE3:   rv =  getCargoSide3BackupPosition3(); break;
            case ROCKET_NEAR:   rv =  getNearRocketBackupPosition3(); break;
            case ROCKET_FAR:    rv =  getFarRocketBackupPosition3(); break;
        }

        return rv;
    }
    
	private static final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			// put("LeftStartPoseX", getHab1LeftStartPose().getX());
            // put("LeftStartPoseY", getHab1LeftStartPose().getY());
            
        
			// put("kCargoSide1TurnPositionX", kCargoSide1TurnPosition.getX());
			// put("kCargoSide1TurnPositionY", kCargoSide1TurnPosition.getY());

            // put("kCargoSide1VisionPosition", kCargoSide1VisionPosition.getX());
            // put("kCargoSide1VisionPosition", kCargoSide1VisionPosition.getY());
            
			// put("kCargoSide1HatchPositionX", kCargoSide1HatchPosition.getX());
			// put("kCargoSide1HatchPositionY", kCargoSide1HatchPosition.getY());

            // put("kCargoSide1BackupPosition", kCargoSide1BackupPosition.getX());
			// put("kCargoSide1BackupPosition", kCargoSide1BackupPosition.getY());
        }
    };
    
    public static DataLogger getLogger() { return logger; }    
}