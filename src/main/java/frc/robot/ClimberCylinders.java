package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;

// TODO: need to have 2 cylinders -- left and right
// need separate control channels for each, but both will extend and retract together

public class ClimberCylinders
{
    private static ClimberCylinders mInstance = null;
    private DoubleSolenoid solenoid;
    private final DoubleSolenoid.Value kSolenoidStateWhenExtended = DoubleSolenoid.Value.kForward; // switch kForward<-->kReverse to reverse the controls polarity
    private DoubleSolenoid.Value kSolenoidStateWhenRetracted;
    private DoubleSolenoid.Value direction;
 

    public synchronized static ClimberCylinders getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new ClimberCylinders();
        }
        return mInstance;
    }

    private ClimberCylinders()
    {
        solenoid = new DoubleSolenoid(Constants.kExample2ForwardChannel, Constants.kExample2ReverseChannel);

        kSolenoidStateWhenRetracted = DoubleSolenoid.Value.kReverse;
        if (kSolenoidStateWhenExtended == DoubleSolenoid.Value.kReverse)
        {
            kSolenoidStateWhenRetracted = DoubleSolenoid.Value.kForward;
        }

        // Start the cylinder in the retracted position. 
        direction = kSolenoidStateWhenExtended; // Set direction to extended to force a state change
        retract();
    }

    // We only call the set() function when the solenoid state changes.
    // This reduces the amount of time required to check the state, and reduces the amount of data on the CAN bus
 
    public synchronized void extend()
    {
        if (direction != kSolenoidStateWhenExtended)
        {
            direction = kSolenoidStateWhenExtended;
            solenoid.set(direction);
        }
    }

    public synchronized void retract()
    {
        if (direction != kSolenoidStateWhenRetracted)
        {
            direction = kSolenoidStateWhenRetracted;
            solenoid.set(direction);
        }
    }

    public synchronized void off()
    {
        if (direction != DoubleSolenoid.Value.kOff)
        {
            direction = DoubleSolenoid.Value.kOff;
            solenoid.set(direction);
        }
    }
}