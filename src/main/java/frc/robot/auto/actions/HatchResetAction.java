package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.command_status.DriveState;
import frc.robot.subsystems.*;

import frc.robot.subsystems.Superstructure;
import frc.robot.HatchDeploy;
import frc.robot.Robot;

/**
 * DriveStraightAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class HatchResetAction implements Action {

    HatchDeploy hatchDeploy = HatchDeploy.getInstance();

    private boolean finished;
  

    public HatchResetAction() {
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
		System.out.println("Starting HatchResetAction");		
        hatchDeploy.retract(); 
        finished = true;
	}

	@Override
	public void done() {
System.out.println("Done HatchResetAction");		

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
