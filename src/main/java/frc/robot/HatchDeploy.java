package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class HatchDeploy {
    public static HatchDeploy mInstance = new HatchDeploy();

    public static HatchDeploy getInstance() {
        return mInstance;
    }

    public TalonSRX dropMotor;
    public Solenoid HatchSolenoid;
    public DigitalInput limitSwitch;
    public final int dropPort = 8;
    public final int hatchPort = 0;
    public final double zeroingSpeed = -0.1;
    public final double startingAngle = 1;
    public final double pickUpAngle = 0.75;
    public final double groundAngle = 0;
    public final double defenseAngle = 0.9;
    boolean on;
    boolean off;

    public enum HatchDeployStateEnum {
        INIT, DEFENSE, TO_BUMPER, GROUND;
    }

    public HatchDeployStateEnum state = HatchDeployStateEnum.INIT;

    public HatchDeploy() {
        HatchSolenoid = new Solenoid(0, hatchPort);
        dropMotor = new TalonSRX(dropPort);
        limitSwitch = new DigitalInput(10);
        state = HatchDeployStateEnum.INIT;
    }

    public void run() {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        switch (state) {
        case INIT:
            dropMotor.set(ControlMode.PercentOutput, zeroingSpeed);
            if (limitSwitch.get()) {
                state = HatchDeployStateEnum.TO_BUMPER;
            }
            break;
        case DEFENSE:
            dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(defenseAngle));
            if (controls.getButton(Constants.kDefenseButton))
            {
                state = HatchDeployStateEnum.TO_BUMPER;
            }
            break;
        case TO_BUMPER:
            dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(pickUpAngle));
            if (controls.getButton(Constants.kCargoIntakeRetractButton)) {
                state = HatchDeployStateEnum.GROUND;
            }
            break;
        case GROUND:
            dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(groundAngle));
            if (controls.getButton(Constants.kCargoIntakeRetractButton)) {
                state = HatchDeployStateEnum.DEFENSE;
            }
            break;
        }

        //shoots both pistons from the solenoid 
        boolean ejectButton = controls.getButton(Constants.kHatchShootButton);
        HatchSolenoid.set(ejectButton);

    }

    public static double dedegsToEncoderUnits(int _encoderUnits)
	{
		return _encoderUnits / Constants.kHatchEncoderUnitsPerDegs;
	}
	
	public static int degsToEncoderUnits(double _inches)
	{
		return (int)(_inches * Constants.kHatchEncoderUnitsPerDegs);
	}
	
	public double encoderVelocityToDegsPerSec(int _encoderVelocity)
	{
		// extra factor of 10 because velocity is reported over 100ms periods 
		return _encoderVelocity * 10.0 / Constants.kHatchEncoderUnitsPerDegs;
	}
    
public void drop() {
    dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(groundAngle));
}
public void deploy(){
    HatchSolenoid.set(on);
}
public void done() {
    HatchSolenoid.set(off);
}

}
