package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.command_status.DriveCommand;

/**
 * An abstract class that forms the base of joystick controls.
 */
public abstract class JoystickControlsBase 
{    
    protected final Joystick[] mStick;
    protected boolean drivingForward = true;
    public static double kJoystickDeadzone = 0.2;   // deadzone at center of joystick extends from +/-kJoystickDeadzone

    public static int kLeftStick   =    0;
    public static int kRightStick  =    1;

    protected JoystickControlsBase() 
    {
        mStick = new Joystick[2];
        mStick[kLeftStick] = new Joystick(0);
        mStick[kRightStick] = new Joystick(1);
    }

    // DRIVER CONTROLS
    public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveSignal
 
    public boolean getButton(int _joystickNum, int _buttonNum)      { return mStick[_joystickNum].getRawButton(_buttonNum); }
    public boolean getAxisAsButton(int _joystickNum, int _axisNum)  { return mStick[_joystickNum].getRawAxis(_axisNum) > 0.5; }
    public int getPOV(int _joystickNum)                             { return mStick[_joystickNum].getPOV(); }

    public boolean getButton(int _buttonNum)       { return mStick[kLeftStick].getRawButton(_buttonNum); }
    public boolean getAxisAsButton(int _axisNum)   { return mStick[kLeftStick].getRawAxis(_axisNum) > 0.5; }
    public int getPOV()                            { return mStick[kLeftStick].getPOV(); }

    public boolean usingLeftStick() { return drivingForward; }
    public boolean joystickActive() { return true; }

    public void setRumble(GenericHID.RumbleType _rumbleType, double _value)       { mStick[kLeftStick].setRumble(_rumbleType, _value); }    
    
}
