package frc.robot.auto.actions;

import frc.robot.HatchDeploy;
import frc.robot.lib.util.DataLogger;

/**
 * DriveStraightAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class HatchEjectAction implements Action {

    HatchDeploy hatchDeploy = HatchDeploy.getInstance();

    private boolean finished;
  

    public HatchEjectAction() {
        finished = false;

    }

    @Override
	public void start() 
	{
	finished = false;
	}

	@Override
	public boolean isFinished() {
		return finished;
	}

	@Override
	public void update() {
		System.out.println("Starting HatchEjectAction");		
        hatchDeploy.eject(); 
        finished = true;
	}

	@Override
	public void done() {
System.out.println("Done HatchEjectAction");		

	}

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
	@Override
	public DataLogger getLogger() { return logger; }

}
