package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.util.RisingEdgeDetector;

public class HatchDeploy {
    public static HatchDeploy mInstance = new HatchDeploy();

    public static HatchDeploy getInstance() {
        return mInstance;
    }

    public TalonSRX dropMotor;
    public Solenoid hatchSolenoid;
    public DigitalInput limitSwitch;
    public final double zeroingSpeed = -0.1;
    public final double startingAngle = 1;
    public final double pickUpAngle = 0.75;
    public final double groundAngle = 0;
    public final double defenseAngle = 0.9;
    public RisingEdgeDetector defenseBttnRisingEdgeDetector = new RisingEdgeDetector();
    boolean on;
    boolean off;


    // math for limit switch
    public static double kEncoderUnitsPerRev = 4096;
    public static double kEncoderDegPerRev = 360;
    public static double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/kEncoderDegPerRev;
    
    
    public enum HatchDeployStateEnum {
        INIT, DEFENSE, TO_BUMPER, GROUND;
    }

    public HatchDeployStateEnum state = HatchDeployStateEnum.INIT;

    public HatchDeploy() {
        hatchSolenoid = new Solenoid(0, Constants.kHatchEjectChannel);
        dropMotor = new TalonSRX(Constants.kHatchDeployTalonId);
        limitSwitch = new DigitalInput(10);
        state = HatchDeployStateEnum.INIT;
    }

    public void run() {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        boolean dBtnIsPushed = controls.getButton(Constants.kDefenseButton);
        boolean dBttnEdgeDetectorValue = defenseBttnRisingEdgeDetector.update(dBtnIsPushed);
        switch (state) {
        case INIT:
            dropMotor.set(ControlMode.PercentOutput, zeroingSpeed);
            if (limitSwitch.get()) {
                state = HatchDeployStateEnum.TO_BUMPER;
            }
            break;
            case TO_BUMPER:
            dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(pickUpAngle));
            if (controls.getButton(Constants.kHatchExtendRetractButton))  // TODO: rising edge of kHatchExtendRetractButton
            {    
                state = HatchDeployStateEnum.GROUND;
            }
            if (dBttnEdgeDetectorValue)
            {
                state = HatchDeployStateEnum.DEFENSE;
            }
            break;
        case DEFENSE:
            dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(defenseAngle));
            if (dBttnEdgeDetectorValue)
            {
                state = HatchDeployStateEnum.TO_BUMPER;
            }
            break;
        case GROUND:
            dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(groundAngle));
            if (controls.getButton(Constants.kHatchExtendRetractButton))    // TODO: rising edge of kHatchExtendRetractButton
             {
                state = HatchDeployStateEnum.TO_BUMPER;
            }
            break;
        }

        //shoots both pistons from the solenoid 
        boolean ejectButton = controls.getAxisAsButton(Constants.kHatchShootAxis);
        hatchSolenoid.set(ejectButton);

    }

    public static double dedegsToEncoderUnits(int _encoderUnits)
	{
		return _encoderUnits / kEncoderUnitsPerDeg;
	}
	
	public static int degsToEncoderUnits(double _inches)
	{
		return (int)(_inches * kEncoderUnitsPerDeg);
	}
	
	public double encoderVelocityToDegsPerSec(int _encoderVelocity)
	{
		// extra factor of 10 because velocity is reported over 100ms periods 
		return _encoderVelocity * 10.0 / kEncoderUnitsPerDeg;
	}
    
public void drop() {
    dropMotor.set(ControlMode.MotionMagic, degsToEncoderUnits(groundAngle));
}
public void deploy(){
    hatchSolenoid.set(on);
}
public void done() {
    hatchSolenoid.set(off);
}

}
