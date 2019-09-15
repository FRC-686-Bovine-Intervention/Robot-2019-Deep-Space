package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class TmReversibleArcadeDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new TmReversibleArcadeDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

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
    }


    boolean usingLeftStick = false;     // default this way so autonomous uses hatch camera
	double kCrossoverThreshold = 0.2;
    boolean leftStickActive = false;
    boolean rightStickActive = false;

	double rThrottle;
	double rTurn;
	double lThrottle;
	double lTurn;
	double throttle;
	double turn;

	public DriveCommand getDriveCommand()
	{      
		rThrottle = -mStick[kRightStick].getRawAxis(kYAxis);
		rTurn     = -mStick[kRightStick].getRawAxis(kXAxis);
		lThrottle = -mStick[kLeftStick].getRawAxis(kYAxis);
		lTurn     = -mStick[kLeftStick].getRawAxis(kXAxis);

		throttle = lThrottle;
		turn = lTurn;


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

// System.out.printf("lThrottle: %5.3f, lTurn: %5.3f, rThrottle: %5.3f, rTurn: %5.3f, lAct:%b, rAct:%b, usingLeft:%b\n", lThrottle, lTurn, rThrottle, rTurn, leftStickActive, rightStickActive, usingLeftStick);

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

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("TmJoystick/throttle", throttle );
            put("TmJoystick/turn", turn );
            for (int stickNum = kLeftStick; stickNum <= kRightStick; stickNum++)
            {
                String stickName = (stickNum == kLeftStick ? "LStick/" : "RStick/");
	    		put("TmJoystick/" + stickName + "xAxis", getAxis(stickNum, kXAxis));
                put("TmJoystick/" + stickName + "yAxis", getAxis(stickNum, kYAxis));
	    		put("TmJoystick/" + stickName + "rotate", getAxis(stickNum, kZRotateAxis));
                put("TmJoystick/" + stickName + "slider", getAxis(stickNum, kSliderAxis));
                int buttons = 0;
                for (int button=1; button<=16; button++)
                {
                    buttons |= (getButton(stickNum, button) ? 1 : 0) << (button-1);
                }
                put("TmJoystick/" + stickName + "buttons", buttons);
                put("TmJoystick/" + stickName + "pov", getPOV(stickNum));
            }
        }
    };
	
    public DataLogger getLogger() { return logger; }

}
