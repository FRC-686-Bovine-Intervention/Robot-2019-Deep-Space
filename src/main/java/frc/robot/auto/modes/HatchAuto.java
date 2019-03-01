package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DeployHatchAction;
import frc.robot.auto.actions.HatchEjectAction;
import frc.robot.auto.actions.HatchResetAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.loops.DriveLoop;
import frc.robot.lib.util.Vector2d;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */


public class HatchAuto extends AutoModeBase {

    public HatchAuto() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        double vel = 36;    //DriveLoop.kPathFollowingMaxVel;
        double accel = 24;  // DriveLoop.kPathFollowingMaxAccel

    PathSegment.Options pathOptions	= new PathSegment.Options(vel, accel, 24, false);
    PathSegment.Options tightTurnOptions	= new PathSegment.Options(vel, accel, 12, false);
    PathSegment.Options visionOptions	= new PathSegment.Options(vel, accel, 24, true);


    Path platformToSide1= new Path();
    platformToSide1.add(new Waypoint(FieldDimensions.getLeftStartPose().getPosition(), pathOptions));
    platformToSide1.add(new Waypoint(FieldDimensions.getCargoShipSideBay1TurnPosition(), pathOptions));
    platformToSide1.add(new Waypoint(FieldDimensions.getCargoShipSideBay1VisionPosition(), tightTurnOptions));
    platformToSide1.add(new Waypoint(FieldDimensions.getCargoShipSideBay1HatchPosition(), visionOptions));

        Path backupCargoPath = new Path();
        backupCargoPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay1HatchPosition(), pathOptions));
        backupCargoPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay1BackupPosition(), pathOptions));
        backupCargoPath.setReverseDirection();

        Path driveToHumanStationPath = new Path();
        driveToHumanStationPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay1BackupPosition(), pathOptions));
        driveToHumanStationPath.add(new Waypoint(FieldDimensions.getHumanStationVisionPosition(), pathOptions));
        driveToHumanStationPath.add(new Waypoint(FieldDimensions.getHumanStationHatchPosition(), visionOptions));

        // Path backUpCargoShipPath = new Path();
        // driveToHumanStationPath.add(new Waypoint(FieldDimensions.getHumanStationVisionPosition(), pathOptions));
        // driveToHumanStationPath.add(new Waypoint(FieldDimensions.getCargoShipBay2BackupPosition(), pathOptions));

        // Path turnBay2Path = new Path();
        // turnCargoShipPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay2TurnPosition(), tightTurnOptions));
        // turnCargoShipPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay2VisionPosition(), tightTurnOptions));
    
        // Path visionBay2Path = new Path();
        // visionBay2Path.add(new Waypoint(FieldDimensions.getCargoShipSideBay2VisionPosition(), pathOptions));
        // visionBay2Path.add(new Waypoint(FieldDimensions.getCargoShipSideBay2HatchPosition(), pathOptions));

        // Path outOfWayPath = new Path();
        // outOfWayPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay2HatchPosition(), pathOptions));
        // outOfWayPath.add(new Waypoint(FieldDimensions.getCargoShipBay2BackupPosition(), pathOptions));

        runAction(new PathFollowerAction(platformToSide1)); //drive off platform 

        runAction(new HatchEjectAction()); //eject hatch action
        runAction(new WaitAction(2));
        runAction(new HatchResetAction());

        runAction(new PathFollowerAction(backupCargoPath)); 
        runAction(new PathFollowerAction(driveToHumanStationPath));// back up to human station
        // runAction(new PathFollowerAction(backUpCargoShipPath));//drive forward to cargoship 
        // runAction(new PathFollowerAction(turnBay2Path));//turn towards  2nd bay
        // runAction(new PathFollowerAction(visionBay2Path)); // turn on vision to line up with reflective and white tape
        // runAction(new HatchEjectAction());  //eject hatch action
        // runAction(new WaitAction(2));
        // runAction(new HatchResetAction());
        // runAction(new PathFollowerAction(outOfWayPath)); //backup a foot to get out of the way
    }
}
