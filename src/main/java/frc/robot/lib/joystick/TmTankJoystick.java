package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Util;

public class TmTankJoystick extends JoystickControlsBase 
{
    // singleton class
     private static JoystickControlsBase instance = null;
     public static JoystickControlsBase getInstance() 
     { 
        if (instance == null) {
            instance = new TmTankJoystick();
            }
        return instance;
    }

    protected final Joystick[] mStick;

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


    public TmTankJoystick() 
    {
        mStick = new Joystick[2];
        mStick[kLeftStick] = new Joystick(0);
        mStick[kRightStick] = new Joystick(1);
    }

    public static final double kDeadband = 0.02;

    double leftStick = 0.0;
    double rightStick = 0.0;
    
    public DriveCommand getDriveCommand()
    {
	    boolean squaredInputs = false;	// set to true to increase fine control while permitting full power

    	leftStick  = -mStick[kLeftStick].getRawAxis(kYAxis);
        rightStick = -mStick[kRightStick].getRawAxis(kYAxis);
        
        leftStick = handleDeadband(leftStick,  kDeadband);
        rightStick= handleDeadband(rightStick, kDeadband);

        DriveCommand signal = new DriveCommand(leftStick, rightStick);
                
        return signal;        
    }

    
    public double handleDeadband(double val, double deadband) 
    {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }



	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            put("TmJoystick/LeftStick", leftStick );
            put("TmJoystick/RightStick", rightStick );
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
