package frc.robot.auto.actions;

import frc.robot.HatchDeploy;
import frc.robot.lib.util.DataLogger;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class HatchCollisionDetectionAction implements Action 
{
    HatchDeploy hatchDeploy = HatchDeploy.getInstance();
    double hatchAngle = Double.MAX_VALUE;
    boolean enable = false;
    boolean collisionDetected = false;
    public static double kEnableAngleThreshold = HatchDeploy.collisionAngle - 50;		// we have to reach this angle before collision detection is enabled
    public static double kCollisionAngleThreshold = HatchDeploy.bumperAngle -  0;		// encoder reading when hatch panel is forced vertical

    public HatchCollisionDetectionAction() 
    {
    }

    @Override
    public void start() 
    {
        enable = false;                                                         // don't allow  detections until hatch has moved forward first
        hatchDeploy.setState(HatchDeploy.HatchDeployStateEnum.AUTO_COLLISION);  // lean hatch forward a bit
        collisionDetected = false;
    }


    @Override
    public void update() 
    {
        // nothing to do.  just waiting for isFinished()
    }	
	
	
    @Override
    public boolean isFinished() 
    {
        hatchAngle = hatchDeploy.getHatchAngle();
        if (hatchAngle >= kEnableAngleThreshold)
        {
            enable = true;
        }
        collisionDetected = enable && (hatchAngle <= kCollisionAngleThreshold);
    	return collisionDetected;
    }

    @Override
    public void done() 
    {
        // cleanup code, if any
        hatchDeploy.setState(HatchDeploy.HatchDeployStateEnum.TO_BUMPER);  // back to usual angle
        
    }

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("HatchCollision/hatchAngle", hatchAngle );
            put("HatchCollision/enableAngleThresh", kEnableAngleThreshold );
            put("HatchCollision/enable", enable );
            put("HatchCollision/collisionAngleThresh", kCollisionAngleThreshold );
            put("HatchCollision/collisionDetected", collisionDetected );
        }
    };
     
    public DataLogger getLogger() { return logger; }
}
