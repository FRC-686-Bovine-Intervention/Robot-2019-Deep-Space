package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.command_status.DriveCommand;

/**
 * An abstract class that forms the base of joystick controls.
 */
// public abstract class JoystickControlsBase 
// {    
//     protected final Joystick mStick;
//     protected boolean drivingForward = true;
//     public static double kJoystickDeadzone = 0.2;   // deadzone at center of joystick extends from +/-kJoystickDeadzone

//     protected JoystickControlsBase(int _port) 
//     {
//         mStick = new Joystick(_port);
//     }

//     // DRIVER CONTROLS
//     public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveSignal
 
//     public double getAxis(int _axisNum)           { return mStick.getRawAxis(_axisNum); }
//     public boolean getAxisAsButton( int _axisNum) { return mStick.getRawAxis(_axisNum) > 0.5; }
//     public boolean getButton(int _buttonNum)      { return mStick.getRawButton(_buttonNum); }
//     public int getPOV()                           { return mStick.getPOV(); }

//     public boolean usingLeftStick() { return drivingForward; }
//     public boolean joystickActive() { return true; }

//     public void setRumble(GenericHID.RumbleType _rumbleType, double _value)       { mStick.setRumble(_rumbleType, _value); }    
    

// }
