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

	// dimensions of field components
	public static double kFieldLengthX = 648;       // 54'
	public static double kFieldLengthY = 324;       // 27'
    
    // Habitat
    public static double kHabWidthY = 128.0;        // not counting ramp
    public static double kHab3DepthX = 48.0;

    // Robot starting poses
	public static Pose centerLeftStartPose  = new Pose(kHab3DepthX + Constants.kCenterToRearBumper, 0,                                            0);
	public static Pose centerRightStartPose = new Pose(kHab3DepthX + Constants.kCenterToRearBumper, 0,                                            0);	
	public static Pose leftStartPose        = new Pose(kHab3DepthX + Constants.kCenterToRearBumper, kHabWidthY/2 - Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB
	public static Pose rightStartPose       = new Pose(kHab3DepthX + Constants.kCenterToRearBumper, kHabWidthY/2 + Constants.kCenterToSideBumper, 0);  // side of robot aligned with edge of HAB

    // Rocket
    public static Vector2d kRocketCenter = new Vector2d(229.1, 162.0);          // TODO: fix this -- right now Y is too high
    public static double   kRocketCenterToHatch = 17.0;                         // distance from rocket center to center of hatches on diagonal
    public static double   kRocketAngleRad = 30.0*Vector2d.degreesToRadians ;   // angle from center to hatch, relative to back
    public static double   kRocketTurnDist = 60.0;                              // distance from which to turn towards rocket
    public static double   kRocketVisionDist = 48.0;                            // distance from which to turn on cameras
    
    public static Vector2d kNearRocketHatchPosition  = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketCenterToHatch, Math.PI-kRocketAngleRad) );
    public static Vector2d kNearRocketTurnPosition   = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketTurnDist,      Math.PI-kRocketAngleRad) );
    public static Vector2d kNearRocketVisionPosition = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketVisionDist,    Math.PI-kRocketAngleRad) );
    public static Vector2d kNearRocketBackupPosition = new Vector2d(180.0, 110.0);  // point to backup to from near rocket hatch

    public static Vector2d kFarRocketHatchPosition   = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketCenterToHatch,     0.0-kRocketAngleRad) );
    public static Vector2d kFarRocketTurnPosition    = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketTurnDist,          0.0-kRocketAngleRad) );
    public static Vector2d kFarRocketVisionPosition  = kRocketCenter.add( Vector2d.magnitudeAngle(kRocketVisionDist,        0.0-kRocketAngleRad) );
    public static Vector2d kFarRocketBackupPosition  = kFarRocketTurnPosition;      // point to backup to from far rocket hatch

    // Cargo Ship
    public static Vector2d kCargoShipFrontBay = new Vector2d(220.3, 10.9);
    public static Vector2d kCargoShipSideBay1 = new Vector2d(260.8, 27.9);
    public static Vector2d kCargoShipSideBay2 = new Vector2d(282.6, 27.9);
    public static Vector2d kCargoShipSideBay3 = new Vector2d(304.3, 27.9);
    public static double   kCargoShipTurnDist   = 60.0;                      // distance from which to turn towards cargo bay
    public static double   kCargoShipVisionDist = 48.0;                      // distance from which to turn turn on cameras (desired target should be mostly centered)

    public static Vector2d kCargoShipFrontBayTurnPosition   = kCargoShipFrontBay.add( Vector2d.magnitudeAngle(kCargoShipTurnDist,   Math.PI) );
    public static Vector2d kCargoShipFrontBayVisionPosition = kCargoShipFrontBay.add( Vector2d.magnitudeAngle(kCargoShipVisionDist, Math.PI) );
    public static Vector2d kCargoShipFrontBayBackupPosition = kCargoShipFrontBay.add(new Vector2d(-30.0, -30.0));

    public static Vector2d kCargoShipSideBay1TurnPosition   = kCargoShipSideBay1.add( Vector2d.magnitudeAngle(kCargoShipTurnDist,   Math.PI/2) );
    public static Vector2d kCargoShipSideBay1VisionPosition = kCargoShipSideBay1.add( Vector2d.magnitudeAngle(kCargoShipVisionDist, Math.PI/2) );
    public static Vector2d kCargoShipSideBay1BackupPosition = kCargoShipSideBay1.add(new Vector2d(+30.0, +30.0));

    public static Vector2d kCargoShipSideBay2TurnPosition   = kCargoShipSideBay2.add( Vector2d.magnitudeAngle(kCargoShipTurnDist,   Math.PI/2) );
    public static Vector2d kCargoShipSideBay2VisionPosition = kCargoShipSideBay2.add( Vector2d.magnitudeAngle(kCargoShipVisionDist, Math.PI/2) );
    public static Vector2d kCargoShipSideBay2BackupPosition = kCargoShipSideBay2.add(new Vector2d(+30.0, +30.0));

    public static Vector2d kCargoShipSideBay3TurnPosition   = kCargoShipSideBay3.add( Vector2d.magnitudeAngle(kCargoShipTurnDist,   Math.PI/2) );
    public static Vector2d kCargoShipSideBay3VisionPosition = kCargoShipSideBay3.add( Vector2d.magnitudeAngle(kCargoShipVisionDist, Math.PI/2) );
    public static Vector2d kCargoShipSideBay3BackupPosition = kCargoShipSideBay3.add(new Vector2d(+30.0, +30.0));
    
    // Human Station
    public static Vector2d kHumanStation = new Vector2d(0.0, 136.3);
    public static double   kHumanStationVisionDist = 96.0;                      // distance from which to turn turn on cameras (desired target should be mostly centered)
    public static Vector2d kHumanStationVisionPosition = kHumanStation.add( Vector2d.magnitudeAngle(kHumanStationVisionDist, 0.0) );



    
    // gets

    // rightSide is set to true if the chosen start pose is on the right
	public static Pose getCenterLeftStartPose()     { rightSide = false;   return centerLeftStartPose; }	
	public static Pose getCenterRightStartPose()    { rightSide =  true;   return centerRightStartPose; }	
	public static Pose getLeftStartPose()           { rightSide = false;   return leftStartPose; }  
    public static Pose getRightStartPose()          { rightSide =  true;   return rightStartPose; } 
    
    // the y-coordinate is negated if the position is on the right side of the field (using complex conjugate)
	public static Vector2d getNearRocketHatchPosition()   { return (rightSide ? kNearRocketHatchPosition  : kNearRocketHatchPosition.conj()); }	
	public static Vector2d getNearRocketTurnPosition()    { return (rightSide ? kNearRocketTurnPosition   : kNearRocketTurnPosition.conj()); }	
	public static Vector2d getNearRocketVisionPosition()  { return (rightSide ? kNearRocketVisionPosition : kNearRocketVisionPosition.conj()); }	
    public static Vector2d getNearRocketBackupPosition()  { return (rightSide ? kNearRocketBackupPosition : kNearRocketBackupPosition.conj()); }	
    
	public static Vector2d getFarRocketHatchPosition()    { return (rightSide ? kFarRocketHatchPosition   : kFarRocketHatchPosition.conj()); }	
	public static Vector2d getFarRocketTurnPosition()     { return (rightSide ? kFarRocketTurnPosition    : kFarRocketTurnPosition.conj()); }	
	public static Vector2d getFarRocketVisionPosition()   { return (rightSide ? kFarRocketVisionPosition  : kFarRocketVisionPosition.conj()); }	
	public static Vector2d getFarRocketBackupPosition()   { return (rightSide ? kFarRocketBackupPosition  : kFarRocketBackupPosition.conj()); }	

    public static Vector2d getCargoShipFrontBayHatchPosition()  { return (rightSide ? kCargoShipFrontBay   : kCargoShipFrontBay.conj()); }	
    public static Vector2d getCargoShipFrontBayTurnPosition()   { return (rightSide ? kCargoShipFrontBayTurnPosition   : kCargoShipFrontBayTurnPosition.conj()); }	
    public static Vector2d getCargoShipFrontBayVisionPosition() { return (rightSide ? kCargoShipFrontBayVisionPosition : kCargoShipFrontBayVisionPosition.conj()); }	
    public static Vector2d getCargoShipFrontBayBackupPosition() { return (rightSide ? kCargoShipFrontBayBackupPosition : kCargoShipFrontBayBackupPosition.conj()); }	
   
    public static Vector2d getCargoShipSideBay1HatchPosition()  { return (rightSide ? kCargoShipSideBay1   : kCargoShipSideBay1.conj()); }	
    public static Vector2d getCargoShipSideBay1TurnPosition()   { return (rightSide ? kCargoShipSideBay1TurnPosition   : kCargoShipSideBay1TurnPosition.conj()); }	
    public static Vector2d getCargoShipSideBay1VisionPosition() { return (rightSide ? kCargoShipSideBay1VisionPosition : kCargoShipSideBay1VisionPosition.conj()); }	
    public static Vector2d getCargoShipSideBay1BackupPosition() { return (rightSide ? kCargoShipSideBay1BackupPosition : kCargoShipSideBay1BackupPosition.conj()); }	
    
    public static Vector2d getCargoShipSideBay2HatchPosition()  { return (rightSide ? kCargoShipSideBay2   : kCargoShipSideBay2.conj()); }	
    public static Vector2d getCargoShipSideBay2TurnPosition()   { return (rightSide ? kCargoShipSideBay2TurnPosition   : kCargoShipSideBay2TurnPosition.conj()); }	
    public static Vector2d getCargoShipSideBay2VisionPosition() { return (rightSide ? kCargoShipSideBay2VisionPosition : kCargoShipSideBay2VisionPosition.conj()); }	
    public static Vector2d getCargoShipSideBay2BackupPosition() { return (rightSide ? kCargoShipSideBay2BackupPosition : kCargoShipSideBay2BackupPosition.conj()); }	
    
    public static Vector2d getCargoShipSideBay3HatchPosition()  { return (rightSide ? kCargoShipSideBay3   : kCargoShipSideBay3.conj()); }	
    public static Vector2d getCargoShipSideBay3TurnPosition()   { return (rightSide ? kCargoShipSideBay3TurnPosition   : kCargoShipSideBay3TurnPosition.conj()); }	
    public static Vector2d getCargoShipSideBay3VisionPosition() { return (rightSide ? kCargoShipSideBay3VisionPosition : kCargoShipSideBay3VisionPosition.conj()); }	
    public static Vector2d getCargoShipSideBay3BackupPosition() { return (rightSide ? kCargoShipSideBay3BackupPosition : kCargoShipSideBay3BackupPosition.conj()); }	
    
    public static Vector2d getHumanStationHatchPosition() { return (rightSide ? kHumanStation : kHumanStation.conj()); }
    public static Vector2d getHumanStationVisionPosition() { return (rightSide ? kHumanStationVisionPosition : kHumanStationVisionPosition.conj()); }



	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("LeftStartPoseX", getLeftStartPose().getX());
            put("LeftStartPoseY", getLeftStartPose().getY());
            
			put("kCargoShipSideBay1TurnPositionX", kCargoShipSideBay1TurnPosition.getX());
			put("kCargoShipSideBay1TurnPositionY", kCargoShipSideBay1TurnPosition.getY());

            put("kCargoShipSideBay1VisionPosition", kCargoShipSideBay1VisionPosition.getX());
            put("kCargoShipSideBay1VisionPosition", kCargoShipSideBay1VisionPosition.getY());
            
			put("kCargoShipSideBay1HatchPositionX", kCargoShipSideBay1.getX());
			put("kCargoShipSideBay1HatchPositionY", kCargoShipSideBay1.getY());

            put("kCargoShipSideBay1BackupPosition", kCargoShipSideBay1BackupPosition.getX());
			put("kCargoShipSideBay1BackupPosition", kCargoShipSideBay1BackupPosition.getY());
        }
    };
    
    public DataLogger getLogger() { return logger; }    
}