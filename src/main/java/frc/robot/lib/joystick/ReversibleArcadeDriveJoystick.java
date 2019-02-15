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

		boolean drivingForward = true;
		double kCrossoverThreshold = 0.2;

		public DriveCommand getDriveCommand()
		{
			double lThrottle = -mStick.getRawAxis(Constants.kXboxLStickYAxis);	
			double lTurn     = -mStick.getRawAxis(Constants.kXboxLStickXAxis);	
			double rThrottle = -mStick.getRawAxis(Constants.kXboxRStickYAxis);
			double rTurn     = -mStick.getRawAxis(Constants.kXboxRStickXAxis);

			double throttle = lThrottle;
			double turn = lTurn;

// System.out.printf("lThrottle: %5.3f, lTurn: %5.3f, rThrottle: %5.3f, rTurn: %5.3f\n", lThrottle, lTurn, rThrottle, rTurn);

			// check to see if we are switching sticks
			if (drivingForward)
			{
				if ((Math.abs(lThrottle) <  kCrossoverThreshold) && (Math.abs(lTurn) <  kCrossoverThreshold) && 
				    (Math.abs(rThrottle) >= kCrossoverThreshold) || (Math.abs(rTurn) >= kCrossoverThreshold))
					drivingForward = false;
			}
			else
			{
				if ((Math.abs(rThrottle) <  kCrossoverThreshold) && (Math.abs(rTurn) <  kCrossoverThreshold) && 
				    (Math.abs(lThrottle) >= kCrossoverThreshold) || (Math.abs(lTurn) >= kCrossoverThreshold))
					drivingForward = true;
				}

			// if we are driving in reverse, flip stick controls
			if (!drivingForward)
			{
				throttle = -rThrottle;
				turn = +rTurn;
			}

			DriveCommand signal = ArcadeDriveJoystick.throttleTurnToDriveCommand(throttle, turn);
	   	    
			return signal; 
		}       
		
		@Override
		public boolean getDrivingForward()
		{
			return drivingForward;
		}

}
