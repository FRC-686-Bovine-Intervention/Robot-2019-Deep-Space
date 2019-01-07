package frc.robot.auto.modes;

import frc.robot.Constants;
import frc.robot.lib.util.Pose;

/**
 * Interface that holds all the field measurements 
 */

//https://www.chiefdelphi.com/forums/attachment.php?attachmentid=22877&d=1516118855

public class FieldDimensions 
{
	// dimensions of field components
	public static double kFieldLengthX = 648;
	public static double kAllianceStationLengthY = 264;
	
	
	// get poses
	public static Pose getCenterStartPose() { return new Pose(Constants.kCenterToRearBumper, 12.0-Constants.kCenterToSideBumper, 0); }	// side of robot aligned with exchange zone tape 
	public static Pose getLeftStartPose()   { return new Pose(Constants.kCenterToRearBumper, (kAllianceStationLengthY/2) - Constants.kCenterToSideBumper, 0); }
	public static Pose getRightStartPose()  { return new Pose(Constants.kCenterToRearBumper, -((kAllianceStationLengthY/2) - Constants.kCenterToSideBumper), 0); }

}
