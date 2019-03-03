package frc.robot.lib.joystick;

import frc.robot.lib.joystick.JoystickControlsBase;

import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and turn.
 */
public class ReversibleArcadeDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new ReversibleArcadeDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    boolean usingLeftStick = false;     // default this way so autonomous uses hatch camera
	double kCrossoverThreshold = 0.2;
    boolean leftStickActive = false;
    boolean rightStickActive = false;

	public DriveCommand getDriveCommand()
	{
		double rThrottle = -mStick.getRawAxis(Constants.kXboxRStickYAxis);
		double rTurn     = -mStick.getRawAxis(Constants.kXboxRStickXAxis);
		double lThrottle = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
		double lTurn     = -mStick.getRawAxis(Constants.kXboxLStickXAxis);

		double throttle = lThrottle;
		double turn = lTurn;

// System.out.printf("lThrottle: %5.3f, lTurn: %5.3f, rThrottle: %5.3f, rTurn: %5.3f\n", lThrottle, lTurn, rThrottle, rTurn);

        leftStickActive =  ((Math.abs(lThrottle) >= kCrossoverThreshold) || (Math.abs(lTurn) >= kCrossoverThreshold));
        rightStickActive = ((Math.abs(rThrottle) >= kCrossoverThreshold) || (Math.abs(rTurn) >= kCrossoverThreshold));

		// check to see if we are switching sticks
		if (usingLeftStick)
		{
			if (rightStickActive && !leftStickActive) {
				usingLeftStick = false;
			}
		}
		else 
		{
			if (leftStickActive && !rightStickActive) {
				usingLeftStick = true;
			}
		}
	
		// if we are driving in reverse, flip stick controls
		if (!usingLeftStick)
		{
			throttle = -rThrottle;
			turn = +rTurn;
		}

		DriveCommand signal = ArcadeDriveJoystick.throttleTurnToDriveCommand(throttle, turn);
		
		return signal; 
	}       
	
	@Override
	public boolean usingLeftStick()
	{
		return usingLeftStick;
	}

	@Override
    public boolean joystickActive()
    {
		// in autonomous, we want to check if driver is attempting to override autonomous
		// by moving joystick.  This is the function that does this.
		
		double rThrottle = -mStick.getRawAxis(Constants.kXboxRStickYAxis);
		double rTurn     = -mStick.getRawAxis(Constants.kXboxRStickXAxis);
		double lThrottle = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
		double lTurn     = -mStick.getRawAxis(Constants.kXboxLStickXAxis);

        leftStickActive =  ((Math.abs(lThrottle) >= kCrossoverThreshold) || (Math.abs(lTurn) >= kCrossoverThreshold));
        rightStickActive = ((Math.abs(rThrottle) >= kCrossoverThreshold) || (Math.abs(rTurn) >= kCrossoverThreshold));

        return leftStickActive || rightStickActive;
	}
}
