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

    PathSegment.Options pathOptions	= new PathSegment.Options(DriveLoop.kPathFollowingMaxVel, DriveLoop.kPathFollowingMaxAccel, 48, false);
    PathSegment.Options tightTurnOptions	= new PathSegment.Options(DriveLoop.kPathFollowingMaxVel/2, DriveLoop.kPathFollowingMaxAccel, 24, false);
    Vector2d StartPosition = new Vector2d(0,0);
    Vector2d turnCargoShipPosition = new Vector2d(0,0);
    Vector2d facingCargoPosition = new Vector2d(0,0);

        Path driveBackWardsOffPlatformPath = new Path();
        driveBackWardsOffPlatformPath.add(new Waypoint(FieldDimensions.getRightStartPose().getPosition(), pathOptions));
        driveBackWardsOffPlatformPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay1TurnPosition(), pathOptions));

        Path turnCargoShipPath = new Path();
        turnCargoShipPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay1TurnPosition(), tightTurnOptions));
        turnCargoShipPath.add(new Waypoint(FieldDimensions.getCargoShipSideBay1VisionPosition(), tightTurnOptions));
    
        Path visionBay1Path = new Path();
        visionBay1Path.add(new Waypoint(FieldDimensions.getCargoShipSideBay1VisionPosition(), tightTurnOptions));
        visionBay1Path.add(new Waypoint(FieldDimensions.getCargoShipSideBay1HatchPosition(), tightTurnOptions));

        runAction(new PathFollowerAction(driveBackWardsOffPlatformPath)); //drive off platform 
        // runAction(new PathfollowerAction(driveToCargoShipPath));//drive to cargoship 
        runAction(new PathFollowerAction(turnCargoShipPath)); //turn towards cargoship 1st bay
        runAction(new PathfollowerAction(visionBay1Path)); // turn on vision to line up with reflective and white tape
        runAction(new HatchEjectAction()); //eject hatch action
        runAction(new WaitAction(2));
        runAction(new HatchResetAction());
        runAction(new PathFollowerAction(backupCargoPath)); 
        runAction(new PathfollowerAction(driveToHumanStationPath));// back up to human station
        runAction(new PathFollowerAction(lineUpHumanStationPath));//turn on vision to line up with reflective and white tape
        runAction(new PathfollowerAction(backUpCargoShipPath));//drive forward to cargoship 
        runAction(new PathFollowerAction(turnBay2Path));//turn towards  2nd bay
        runAction(new PathfollowerAction(visionBay2Path)); // turn on vision to line up with reflective and white tape
        runAction(new HatchEjectAction());  //eject hatch action
        runAction(new WaitAction(2));
        runAction(new HatchResetAction());
        runAction(new PathFollowerAction(outOfWayPath)); //backup a foot to get out of the way
    }
}
