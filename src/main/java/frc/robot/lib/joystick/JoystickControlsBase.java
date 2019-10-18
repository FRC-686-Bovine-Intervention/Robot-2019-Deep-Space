package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.command_status.DriveCommand;

/**
 * An abstract class that forms the base of joystick controls.
 */
public abstract class JoystickControlsBase 
{    
    protected final Joystick[] mStick = new Joystick[2];
    protected boolean drivingForward = true;
    public static double kJoystickDeadzone = 0.1;   // deadzone at center of joystick extends from +/-kJoystickDeadzone

    public static int kLeftStick   =    0;
    public static int kRightStick  =    1;

    protected JoystickControlsBase() 
    {
        mStick[kLeftStick] = new Joystick(0);
        mStick[kRightStick] = new Joystick(1);
    }

    // DRIVER CONTROLS
    public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveSignal
 
    public double getAxis(int _joystickNum, int _axisNum)          { return mStick[_joystickNum].getRawAxis(_axisNum); }
    public boolean getAxisAsButton(int _joystickNum, int _axisNum)  { return mStick[_joystickNum].getRawAxis(_axisNum) > 0.5; }
    public boolean getButton(int _joystickNum, int _buttonNum)      { return mStick[_joystickNum].getRawButton(_buttonNum); }
    public int getPOV(int _joystickNum)                             { return mStick[_joystickNum].getPOV(); }

    public double getAxis(int _axisNum)           { return getAxis(kLeftStick, _axisNum); }
    public boolean getAxisAsButton(int _axisNum)   { return getAxisAsButton(kLeftStick, _axisNum); }
    public boolean getButton(int _buttonNum)       { return getButton(kLeftStick, _buttonNum); }
    public int getPOV()                            { return getPOV(kLeftStick); }

    public boolean usingLeftStick() { return drivingForward; }
    public boolean joystickActive() { return true; }

    public void setRumble(GenericHID.RumbleType _rumbleType, double _value)       { mStick[kLeftStick].setRumble(_rumbleType, _value); }    
    

}
