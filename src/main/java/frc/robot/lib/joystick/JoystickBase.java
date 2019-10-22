package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickBase extends Joystick
{
    // constructor
    public JoystickBase(int _port)
    {
        super(_port);
    }

    public static class ThrottleTurn
    {
        public double throttle;
        public double turn;
    
        public ThrottleTurn()
		{
            this(0.0, 0.0);
        }
        
        public ThrottleTurn(double _throttle, double _turn)
		{
            throttle = _throttle;
            turn = _turn;
		}
    }

    public boolean getButton(int _button)
    {
        return getRawButton(_button);
    }

    public boolean getButtonPressed(int _button)
    {
        return getRawButtonPressed(_button);
    }

    public boolean getButtonReleased(int _button)
    {
        return getRawButtonReleased(_button);
    }

    public double getAxis(int _axis)
    {
        return getRawAxis(_axis);
    }

}