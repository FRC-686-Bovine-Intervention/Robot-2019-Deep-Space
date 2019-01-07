package frc.robot.lib.joystick;

import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.util.Util;
import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;

/**
 * Implements Team 254's Cheesy Drive. "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class CheesyTwoStickDriveJoystick extends JoystickControlsBase 
{
    // singleton class
     private static JoystickControlsBase instance = null;
     public static JoystickControlsBase getInstance() 
     { 
        if (instance == null) {
            instance = new CheesyTwoStickDriveJoystick();
            }
        return instance;
    }

    double mQuickStopAccumulator;
    public static final double kThrottleDeadband = 0.02;
    private static final double kturnDeadband = 0.02;
    private static final double kTurnSensitivity = 1.02;
    private DriveCommand mSignal = new DriveCommand(0, 0);

    
    public DriveCommand getDriveCommand()
    {
    	double throttle = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
        double turn     = +mStick.getRawAxis(Constants.kXboxRStickXAxis);
        boolean isQuickTurn = mStick.getRawButton(Constants.kXboxButtonRB);
        
        throttle = handleDeadband(throttle, kThrottleDeadband);
        turn     = handleDeadband(turn,     kturnDeadband);

        double overPower;
        double angularPower;

        if (isQuickTurn) 
        {
            if (Math.abs(throttle) < 0.2) 
            {
                double alpha = 0.1;
                mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * Util.limit(turn, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = turn;
        }
        else
        {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * turn * kTurnSensitivity - mQuickStopAccumulator;
            if (mQuickStopAccumulator > 1)
                mQuickStopAccumulator -= 1;
            else if (mQuickStopAccumulator < -1)
                mQuickStopAccumulator += 1;
            else
                mQuickStopAccumulator = 0.0;
        }

        double rMotorSpeed = throttle - angularPower;
        double lMotorSpeed = throttle + angularPower;
        
        // scale motor power to keep within limits
        if (lMotorSpeed > 1.0) {
            rMotorSpeed -= overPower * (lMotorSpeed - 1.0);
            lMotorSpeed = 1.0;
        } else if (rMotorSpeed > 1.0) {
            lMotorSpeed -= overPower * (rMotorSpeed - 1.0);
            rMotorSpeed = 1.0;
        } else if (lMotorSpeed < -1.0) {
            rMotorSpeed += overPower * (-1.0 - lMotorSpeed);
            lMotorSpeed = -1.0;
        } else if (rMotorSpeed < -1.0) {
            lMotorSpeed += overPower * (-1.0 - rMotorSpeed);
            rMotorSpeed = -1.0;
        }
        
        mSignal.setMotors( lMotorSpeed, rMotorSpeed );
       
	    return mSignal;        
    }

    
    public double handleDeadband(double val, double deadband) 
    {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

}
