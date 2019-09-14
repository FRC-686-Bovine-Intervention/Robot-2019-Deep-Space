package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and turn.
 */
public class TmReversibleArcadeDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new ReversibleArcadeDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    protected final Joystick[] mStick;

    public static int kLeftStick   =    0;
    public static int kRightStick  =    1;

    public static int kXAxis  =         0;
    public static int kYAxis=           1;
    public static int kZRotateAxis =    2;
    public static int kSliderAxis =     3;

    public static int kTriggerButton =      1;
    public static int kBottomThumbButton =  2;
    public static int kLeftThumbButton =    3;
    public static int kRightThumbButton =   4;

    // counting from inner (or thumb) side
    public static int kTopButton1 =      5;
    public static int kTopButton2 =      6;
    public static int kTopButton3 =      7;
    public static int kBottomButton3 =   8;
    public static int kBottomButton2 =   9;
    public static int kBottomButton1 =  10;
    public static int kTopButton6 =     11;
    public static int kTopButton5 =     12;
    public static int kTopButton4 =     13;
    public static int kBottomButton4 =  14;
    public static int kBottomButton5 =  15;
    public static int kBottomButton6 =  16;

    public TmReversibleArcadeDriveJoystick() 
    {
        mStick = new Joystick[2];
        mStick[kLeftStick] = new Joystick(0);
        mStick[kRightStick] = new Joystick(1);
    }


    boolean usingLeftStick = false;     // default this way so autonomous uses hatch camera
	double kCrossoverThreshold = 0.2;
    boolean leftStickActive = false;
    boolean rightStickActive = false;

	public DriveCommand getDriveCommand()
	{
		double rThrottle = -mStick[kRightStick].getRawAxis(kYAxis);
		double rTurn     = -mStick[kRightStick].getRawAxis(kXAxis);
		double lThrottle = -mStick[kLeftStick].getRawAxis(kYAxis);
		double lTurn     = -mStick[kLeftStick].getRawAxis(kXAxis);

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
		
		double rThrottle = -mStick[kRightStick].getRawAxis(kYAxis);
		double rTurn     = -mStick[kRightStick].getRawAxis(kXAxis);
		double lThrottle = -mStick[kLeftStick].getRawAxis(kYAxis);
		double lTurn     = -mStick[kLeftStick].getRawAxis(kXAxis);

        leftStickActive =  ((Math.abs(lThrottle) >= kCrossoverThreshold) || (Math.abs(lTurn) >= kCrossoverThreshold));
        rightStickActive = ((Math.abs(rThrottle) >= kCrossoverThreshold) || (Math.abs(rTurn) >= kCrossoverThreshold));

        return leftStickActive || rightStickActive;
	}
}
