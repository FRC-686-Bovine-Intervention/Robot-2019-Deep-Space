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


        // runAction(new DriveStraightAction(0, 0)); can set drive parameters this way if driving straight 
        //drive off platform 
        //drive to cargoship - john
        //turn towards cargoship - john
        // turn on vision & line sensor to line up with reflective and white tape
        //drive forward to close distance
        //deploy hatch action
        // turn over to john to get another hatch and drive to ship
        //turn on vision & line sensor to line up with reflective and white tape
        //drive forward to close distance
        //deploy hatch action
        // runAction(new DeployHatchAction());
        //  runAction(new HatchResetAction());
        //  runAction(new PathFollowerAction(turnStationPath));    
        //  runAction(new PathFollowerAction(driveToHatchPath)); 
        //  runAction(new PathFollowerAction(backupPath));
        //  runAction(new PathFollowerAction(turnAroundPath)); 
        //  runAction(new PathFollowerAction(driveToShipPath));
        //  runAction(new lineupAction());		         
    }
}
