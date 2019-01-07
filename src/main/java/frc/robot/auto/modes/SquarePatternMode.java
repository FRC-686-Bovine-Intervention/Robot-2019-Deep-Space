package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.DriveLoop;

/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class SquarePatternMode extends AutoModeBase {

	Path path;
    public SquarePatternMode(int lane, boolean shouldDriveBack) 
    {
 
    }
    
    private void init(){
    	
    	PathSegment.Options options = new PathSegment.Options(DriveLoop.kPathFollowingMaxVel, DriveLoop.kPathFollowingMaxAccel, DriveLoop.kPathFollowingLookahead, false);
    	
    	path = new Path();
    	path.add(new Waypoint(new Vector2d( 0, 0), options));
        path.add(new Waypoint(new Vector2d( 240.0, 0), options));
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");

    	init();
    	
        //Path path = new Path();
        //path.add(new Waypoint(new Vector2d( 0, 0), options));
        //path.add(new Waypoint(new Vector2d( 240.0, 0), options));
        //path.add(new Waypoint(new Vector2d( 96.0, 72.0), options));
        //path.add(new Waypoint(new Vector2d( 0, 72.0), options));
        //path.add(new Waypoint(new Vector2d( 0, 0), options));
  
        //Path revPath = new Path(path);
        //revPath.setReverseOrder();
        //revPath.setReverseDirection();
        
        runAction(new PathFollowerAction(path));			// drive forward
        //runAction(new PathFollowerWithVisionAction(revPath));    	// drive reversed 
    }
}
