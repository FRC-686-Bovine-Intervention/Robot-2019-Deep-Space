package frc.robot.lib.joystick;

import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.util.Util;
import frc.robot.command_status.DriveCommand;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and turn.
 */
public class PolarArcadeDriveJoystick extends JoystickControlsBase 
{
    // singleton class
		private static JoystickControlsBase instance = null;
		public static JoystickControlsBase getInstance() 
		{ 
			 if (instance == null) {
					 instance = new PolarArcadeDriveJoystick();
					 }
			 return instance;
	 }

    
    public DriveCommand getDriveCommand()
    {
	    boolean squaredInputs = false;	// set to true to increase fine control while permitting full power
	    
	    double turn = 0;
	    double throttle = 0;
	    
    	if (-mStick.getY() > 0)
    	{
    		double theta = Math.atan(-mStick.getX()/-mStick.getY());
    		throttle = Math.sqrt(mStick.getY() * mStick.getY() + mStick.getX() * mStick.getX());
    		turn = theta*2/Math.PI;
    	}
    	else if (-mStick.getY() < 0)
    	{
    		double theta = Math.atan(-(-mStick.getX()/-mStick.getY()));
    		throttle = -Math.sqrt(mStick.getY() * mStick.getY() + mStick.getX() * mStick.getX());
    		turn = theta*2/Math.PI;
    	}
    	else
    	{
    		throttle = Math.abs(mStick.getX());
			turn = mStick.getX();
    	}
     
	    double moveValue   = Util.limit(throttle, 1.0);
	    double rotateValue = Util.limit(turn,     1.0);
	    double lMotorSpeed, rMotorSpeed;
	    
	    if (squaredInputs) {
	      // square the inputs (while preserving the sign) to increase fine control
	      // while permitting full power
	      if (moveValue >= 0.0) {
	        moveValue = (moveValue * moveValue);
	      } else {
	        moveValue = -(moveValue * moveValue);
	      }
	      if (rotateValue >= 0.0) {
	        rotateValue = (rotateValue * rotateValue);
	      } else {
	        rotateValue = -(rotateValue * rotateValue);
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
}
