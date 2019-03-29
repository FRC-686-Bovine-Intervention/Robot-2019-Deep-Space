package frc.robot.auto.actions;

import frc.robot.Hatch;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Drive;

/**
 * DriveStraightAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class HatchActionExtend implements Action {

    Hatch hatch = Hatch.getInstance();

    private boolean finished;
  

    public HatchActionExtend() {
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
		System.out.println("Starting HatchActionOpen");		
        hatch.extend(); 
        finished = true;
	}

	@Override
	public void done() {
System.out.println("Done HatchActionOpen");		

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
