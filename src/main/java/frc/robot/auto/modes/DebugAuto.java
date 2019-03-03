package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.HatchEjectAction;
import frc.robot.auto.actions.HatchResetAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

public class DebugAuto extends AutoModeBase {

    public DebugAuto() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        double vel = 24;    //DriveLoop.kPathFollowingMaxVel;
        double accel = 24;  // DriveLoop.kPathFollowingMaxAccel

        PathSegment.Options pathOptions	= new PathSegment.Options(vel, accel, 24, false);
        PathSegment.Options tightTurnOptions	= new PathSegment.Options(vel, accel, 12, false);
        PathSegment.Options visionOptions	= new PathSegment.Options(vel, accel, 24, true);

        // override initial position from SmartDashboardInteractions
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), DriveState.getInstance().getLeftDistanceInches(), DriveState.getInstance().getRightDistanceInches(), new Pose());
    

        Path path1= new Path();
        path1.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
        path1.add(new Waypoint(new Vector2d(30, 0), pathOptions));

        Path path2= new Path();
        path2.add(new Waypoint(new Vector2d(30, 0), tightTurnOptions));
        path2.add(new Waypoint(new Vector2d( 0,-30), visionOptions));
        path2.add(new Waypoint(new Vector2d( 0,-72), visionOptions));
        path2.setReverseDirection();

        Path path3= new Path();
        path3.add(new Waypoint(new Vector2d(  0,-72), pathOptions));
        path3.add(new Waypoint(new Vector2d(  0,-60), pathOptions));


		Limelight.getCargoInstance().setLEDMode(Limelight.LedMode.kOff);
		Limelight.getHatchInstance().setLEDMode(Limelight.LedMode.kOn);

        runAction(new PathFollowerAction(path1)); 
        runAction(new PathFollowerAction(path2)); 
        
        runAction(new HatchEjectAction()); //eject hatch action

        // backup and retract pistons
        double retractDelay = 0.5;
        Action waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
        runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(path3), waitAndRetractAction)));   // reverse away from target

    }
}
