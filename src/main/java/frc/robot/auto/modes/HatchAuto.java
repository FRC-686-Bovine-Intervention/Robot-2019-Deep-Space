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

    // PathSegment.Options pathOptions	= new PathSegment.Options(DriveLoop.kPathFollowingMaxVel, DriveLoop.kPathFollowingMaxAccel, 48, false);
    // PathSegment.Options tightTurnOptions	= new PathSegment.Options(DriveLoop.kPathFollowingMaxVel/2, DriveLoop.kPathFollowingMaxAccel, 24, false);
    // Vector2d StartPosition = new Vector2d(0,0);
    // Vector2d turnCargoShipPosition = new Vector2d(0,0);
    // Vector2d facingCargoPosition = new Vector2d(0,0);

    //     Path driveBackWardsOffPlatformPath = new Path();
    //     driveBackWardsOffPlatformPath.add(new Waypoint(StartPosition, pathOptions));
    //     driveBackWardsOffPlatformPath.add(new Waypoint(turnCargoShipPosition, pathOptions));

    //     Path turnCargoShipPath = new Path();
    //     turnCargoShipPath.add(new Waypoint(turnCargoShipPosition, tightTurnOptions));
    //     turnCargoShipPath.add(new Waypoint(facingCargoPosition, tightTurnOptions));

    //     runAction(new PathFollowerAction(driveBackWardsOffPlatformPath));
    //     runAction(new PathFollowerAction(turnCargoShipPath));
        runAction(new HatchEjectAction());
        runAction(new WaitAction(2));
        runAction(new HatchResetAction());

        // runAction(new DriveStraightAction(0, 0)); can set drive parameters this way if driving straight 
        //drive off platform 
        //drive to cargoship - john
        //turn towards cargoship - john
        // turn on vision & line sensor to line up with reflective and white tape
        //drive forward to close distance
        //deploy hatch action
        // turn over to john to get another hatch and drive to ship
        //turn on vision & line sensor to line up with reflective and white tape
        //drive forward to close distance
        //deploy hatch action         
    }
}
