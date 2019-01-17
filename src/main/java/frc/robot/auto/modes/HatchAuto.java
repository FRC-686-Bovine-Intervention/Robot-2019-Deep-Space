package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
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
         runAction(new DeployHatchAction());
         runAction(new HatchResetAction());
         runAction(new PathFollowerAction(turnStationPath));    
         runAction(new PathFollowerAction(driveToHatchPath)); 
         runAction(new PathFollowerAction(backupPath));
         runAction(new PathFollowerAction(turnAroundPath)); 
         runAction(new PathFollowerAction(driveToShipPath));
         runAction(new lineupAction());		         
    }
}
