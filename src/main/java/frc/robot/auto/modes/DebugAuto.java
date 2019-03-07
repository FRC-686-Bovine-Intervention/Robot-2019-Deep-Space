package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
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
        double vel = 36;    //DriveLoop.kPathFollowingMaxVel;
        double accel = 24;  // DriveLoop.kPathFollowingMaxAccel

        PathSegment.Options pathOptions	= new PathSegment.Options(vel, accel, 48, false);
        PathSegment.Options tightTurnOptions	= new PathSegment.Options(vel, accel, 24, false);
        PathSegment.Options visionOptions	= new PathSegment.Options(vel, accel, 24, true);

        // override initial position from SmartDashboardInteractions
        RobotState.getInstance().reset(new Pose(0,0,Math.PI));
    

        Path path1= new Path(vel);
        path1.add(new Waypoint(new Vector2d( 0,0), visionOptions));
        path1.add(new Waypoint(new Vector2d(96,0), visionOptions));
        path1.setReverseDirection();
        
        // Path path2= new Path();
        // path2.add(new Waypoint(new Vector2d(48,0), visionOptions));
        // path2.add(new Waypoint(new Vector2d(96,0), visionOptions));
        // path2.setReverseDirection();
        
        Path path3 = new Path();
        path3.add(new Waypoint(new Vector2d(96,0), pathOptions));
        path3.add(new Waypoint(new Vector2d(72,0), pathOptions));

		Limelight.getCargoInstance().setLEDMode(Limelight.LedMode.kOff);
		Limelight.getHatchInstance().setLEDMode(Limelight.LedMode.kOn);

        runAction(new WaitAction(5.0)); 
 

        runAction(new PathFollowerAction(path1)); 
        runAction(new InterruptableAction(new HatchCollisionDetectionAction(), new PathFollowerAction(path1))); 
        // runAction(new HatchCollisionDetectionAction()); 
        runAction(new HatchEjectAction()); //eject hatch action

        // backup and retract pistons
        double retractDelay = 0.5;
        Action waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
        // runAction(waitAndRetractAction);
        runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(path3), waitAndRetractAction)));   // reverse away from target

    }
}
