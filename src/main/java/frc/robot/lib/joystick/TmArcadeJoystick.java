package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Util;

public class TmArcadeJoystick extends JoystickControlsBase 
{
    // singleton class
     private static JoystickControlsBase instance = null;
     public static JoystickControlsBase getInstance() 
     { 
        if (instance == null) {
            instance = new TmArcadeJoystick();
            }
        return instance;
    }

    protected final Joystick[] mStick = new Joystick[2];

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


    public TmArcadeJoystick() 
    {
        mStick[kLeftStick] = new Joystick(0);
        mStick[kRightStick] = new Joystick(1);
    }

    double throttle = 0.0;
    double turn = 0.0;
	static double kDeadband = 0.05;

    public DriveCommand getDriveCommand()
    {
	    boolean squaredInputs = false;	// set to true to increase fine control while permitting full power

    	throttle = -mStick[kLeftStick].getRawAxis(kYAxis);
        turn     = +mStick[kLeftStick].getRawAxis(kXAxis);
 
        DriveCommand signal = throttleTurnToDriveCommand(throttle, turn);

        return signal;
    }

    public static DriveCommand throttleTurnToDriveCommand(double throttle, double turn) {
        boolean squaredInputs = true; // set to true to increase fine control while permitting full power

        if (throttle < kJoystickDeadzone && throttle > -kJoystickDeadzone) {
            throttle = 0;
        }
        if (turn < kJoystickDeadzone && turn > -kJoystickDeadzone) {
            turn = 0;
        }

        double moveValue = applyDeadband(throttle);
        double rotateValue = applyDeadband(turn);
        double lMotorSpeed, rMotorSpeed;

        // if (squaredInputs) {
        // 	// square the inputs (while preserving the sign) to increase fine control
        // 	// while permitting full power
        // 	if (moveValue >= 0.0) {
        // 		moveValue = (moveValue * moveValue);
        // 	} else {
        // 		moveValue = -(moveValue * moveValue);
        // 	}
        // 	if (rotateValue >= 0.0) {
        // 		rotateValue = (rotateValue * rotateValue);
        // 	} else {
        // 		rotateValue = -(rotateValue * rotateValue);
        // 	}
        // }
        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control
            // while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue*moveValue);
            } else {
                moveValue = (moveValue * moveValue*moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue*rotateValue);
            } else {
                rotateValue = (rotateValue * rotateValue*rotateValue);
            }
        }


        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                lMotorSpeed = moveValue - rotateValue;
                rMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                lMotorSpeed = Math.max(moveValue, -rotateValue);
                rMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                lMotorSpeed = -Math.max(-moveValue, rotateValue);
                rMotorSpeed = moveValue + rotateValue;
            } else {
                lMotorSpeed = moveValue - rotateValue;
                rMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);

        return signal;
    }

    static double applyDeadband(double _in)
    {
        double sign = Math.signum(_in);							// get sign
        double mag = Math.abs(_in);								// get magnitude
        double out = 1.0/(1.0-kDeadband) * mag - kDeadband;		// inputs from kDeadband to 1.0 are scaled to outputs from 0 to 1
        out = sign * out;										// reapply the sign
        out = Util.limit(out, 1.0);								// limit to maximum of +/-1
        return out;
    }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("TmJoystick/Throttle", throttle );
            put("TmJoystick/Turn", turn );
            for (int stickNum = kLeftStick; stickNum <= kRightStick; stickNum++)
            {
                String stickName = (stickNum == kLeftStick ? "LStick/" : "RStick/");
	    		put("TmJoystick/" + stickName + "xAxis", mStick[stickNum].getRawAxis(kXAxis));
                put("TmJoystick/" + stickName + "yAxis", mStick[stickNum].getRawAxis(kYAxis));
	    		put("TmJoystick/" + stickName + "rotate", mStick[stickNum].getRawAxis(kZRotateAxis));
                put("TmJoystick/" + stickName + "slider", mStick[stickNum].getRawAxis(kSliderAxis));
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
