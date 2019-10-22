package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class TmReversibleArcadeDriveJoystick 
{
    JoystickBase lStick;
    JoystickBase rStick;

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

    public TmReversibleArcadeDriveJoystick(JoystickBase _lStick, JoystickBase _rStick) 
    {
        lStick = _lStick;   // port selected in DriverControls
        rStick = _rStick;
    }

	public DriveCommand getDriveCommand()
	{      
		JoystickBase.ThrottleTurn throttleTurn = getThrottleTurn();
		DriveCommand signal = ArcadeDriveJoystick.throttleTurnToDriveCommand(throttleTurn.throttle, throttleTurn.turn);
		return signal; 
	}       

    public JoystickBase.ThrottleTurn getThrottleTurn()
    {
		rThrottle = -rStick.getAxis(Thrustmaster.kYAxis);
		rTurn     = -rStick.getAxis(Thrustmaster.kXAxis);
		lThrottle = -lStick.getAxis(Thrustmaster.kYAxis);
		lTurn     = -lStick.getAxis(Thrustmaster.kXAxis);
        
        JoystickBase.ThrottleTurn out = new JoystickBase.ThrottleTurn();
		out.throttle = lThrottle;
		out.turn = lTurn;


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
			out.throttle = -rThrottle;
			out.turn = +rTurn;
        }       
        
// System.out.printf("lThrottle: %5.3f, lTurn: %5.3f, rThrottle: %5.3f, rTurn: %5.3f, lAct:%b, rAct:%b, usingLeft:%b\n", lThrottle, lTurn, rThrottle, rTurn, leftStickActive, rightStickActive, usingLeftStick);

        return out;
    }

	public boolean usingLeftStick()
	{
		return usingLeftStick;
	}

    public boolean joystickActive()
    {
		// in autonomous, we want to check if driver is attempting to override autonomous
		// by moving joystick.  This is the function that does this.
		
		JoystickBase.ThrottleTurn throttleTurn = getThrottleTurn();

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
            // for (int stickNum = kLeftStick; stickNum <= kRightStick; stickNum++)
            // {
            //     String stickName = (stickNum == kLeftStick ? "LStick/" : "RStick/");
	    	// 	put("TmJoystick/" + stickName + "xAxis", getAxis(stickNum, kXAxis));
            //     put("TmJoystick/" + stickName + "yAxis", getAxis(stickNum, kYAxis));
	    	// 	put("TmJoystick/" + stickName + "rotate", getAxis(stickNum, kZRotateAxis));
            //     put("TmJoystick/" + stickName + "slider", getAxis(stickNum, kSliderAxis));
            //     int buttons = 0;
            //     for (int button=1; button<=16; button++)
            //     {
            //         buttons |= (getButton(stickNum, button) ? 1 : 0) << (button-1);
            //     }
            //     put("TmJoystick/" + stickName + "buttons", buttons);
            //     put("TmJoystick/" + stickName + "pov", getPOV(stickNum));
            // }
        }
    };
	
    public DataLogger getLogger() { return logger; }

}
