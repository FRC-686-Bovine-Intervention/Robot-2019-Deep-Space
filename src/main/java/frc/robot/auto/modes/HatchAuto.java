package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveStraightAction;
import frc.robot.HatchDeploy;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class HatchAuto extends AutoModeBase {

    public HatchAuto(int lane, boolean shouldDriveBack) 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	 runAction(new DriveStraightAction(0, 0));       		         
    }
}
