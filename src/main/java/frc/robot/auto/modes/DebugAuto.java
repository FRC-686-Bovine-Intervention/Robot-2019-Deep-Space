package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.HatchEjectAction;
import frc.robot.auto.actions.HatchResetAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */


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

        RobotState.getInstance().reset(Timer.getFPGATimestamp(), DriveState.getInstance().getLeftDistanceInches(), DriveState.getInstance().getRightDistanceInches(), new Pose());
    

        Path path1= new Path();
        path1.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
        path1.add(new Waypoint(new Vector2d(48, 0), pathOptions));
        path1.add(new Waypoint(new Vector2d(48,-48), pathOptions));
    
        Path path2= new Path();
        path2.add(new Waypoint(new Vector2d(48,-48), pathOptions));
        path2.add(new Waypoint(new Vector2d(96,0), pathOptions));
        path2.setReverseDirection();


        Path path3= new Path();
        path3.add(new Waypoint(new Vector2d(96,0), pathOptions));
        path3.add(new Waypoint(new Vector2d(0,48), pathOptions));
    
        Path path4= new Path(); 
        path4.add(new Waypoint(new Vector2d(0,48), pathOptions));
        path4.add(new Waypoint(new Vector2d(48,0), pathOptions));
        path4.setReverseDirection();

        Path path5= new Path();
        path5.add(new Waypoint(new Vector2d(48,0), pathOptions));
        path5.add(new Waypoint(new Vector2d(0,0), pathOptions));

        runAction(new PathFollowerAction(path1)); 

        runAction(new HatchEjectAction()); //eject hatch action
        runAction(new WaitAction(1));
        runAction(new HatchResetAction());

        runAction(new PathFollowerAction(path2)); 
        runAction(new PathFollowerAction(path3)); 
        runAction(new PathFollowerAction(path4)); 
        runAction(new PathFollowerAction(path5)); 
    }
}
