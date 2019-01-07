package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;

/**
 * An abstract class that forms the base of joystick controls.
 */
public class ButtonBoard 
{
 	// singleton class
	 private static ButtonBoard instance = null;
	 public static ButtonBoard getInstance() 
	 { 
			 if (instance == null) {
					 instance = new ButtonBoard();
			 }
			 return instance;
	 }

    protected final Joystick mStick;

    protected ButtonBoard() 
    {
        mStick = new Joystick(1);
    }

    public boolean getButton(int _num) { return mStick.getRawButton(_num); }

    public int getPOV() 
    {
    	return mStick.getPOV();
    }    
}
