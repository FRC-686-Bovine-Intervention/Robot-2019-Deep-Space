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

	 public static int kButtonBoardA = 1;
	 public static int kButtonBoardB = 2;
	 public static int kButtonBoardX = 3;
	 public static int kButtonBoardY = 4;
	 public static int kButtonBoardLB = 5;
	 public static int kButtonBoardRB = 6;
	 public static int kButtonBoardShare = 7;
	 public static int kButtonBoardOptions = 8;
	 public static int kButtonBoardSL = 9;
	 public static int kButtonBoardSR = 10;
 
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
