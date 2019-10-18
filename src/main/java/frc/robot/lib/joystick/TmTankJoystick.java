package frc.robot.lib.joystick;

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


    public TmTankJoystick() 
    {
    }

    public static final double kDeadband = 0.02;

    double leftStick = 0.0;
    double rightStick = 0.0;
    
    public DriveCommand getDriveCommand()
    {
	    boolean squaredInputs = false;	// set to true to increase fine control while permitting full power

    	leftStick  = -mStick[kLeftStick].getRawAxis(ThrustMasterConstants.kYAxis);
        rightStick = -mStick[kRightStick].getRawAxis(ThrustMasterConstants.kYAxis);
        
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
	    		put("TmJoystick/" + stickName + "xAxis", mStick[stickNum].getRawAxis(ThrustMasterConstants.kXAxis));
                put("TmJoystick/" + stickName + "yAxis", mStick[stickNum].getRawAxis(ThrustMasterConstants.kYAxis));
	    		put("TmJoystick/" + stickName + "rotate", mStick[stickNum].getRawAxis(ThrustMasterConstants.kZRotateAxis));
                put("TmJoystick/" + stickName + "slider", mStick[stickNum].getRawAxis(ThrustMasterConstants.kSliderAxis));
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
