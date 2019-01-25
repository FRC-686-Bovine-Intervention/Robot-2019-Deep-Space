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
    public final int dropPort = 4;
    public final int hatchPort = 0;
    public final double zeroingSpeed = -0.1;
    public final double startingAngle = 1;
    public final double pickUpAngle = 0.75;
    public final double groundAngle = 0;
    public final double defenseAngle = 0.9;

    public enum HatchDeployStateEnum {
        INIT, DEFENSE, TO_BUMPER, GROUND;
    }

    public HatchDeployStateEnum state = HatchDeployStateEnum.INIT;

    public HatchDeploy() {
        HatchSolenoid = new Solenoid(0, hatchPort);
        dropMotor = new TalonSRX(dropPort);
        limitSwitch = new DigitalInput(5);
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
            dropMotor.set(ControlMode.MotionMagic, inchesToEncoderUnits(defenseAngle));
            if (controls.getButton(Constants.kBumperButton))
            {
                state = HatchDeployStateEnum.TO_BUMPER;
            }
            break;
        case TO_BUMPER:
            dropMotor.set(ControlMode.MotionMagic, inchesToEncoderUnits(pickUpAngle));
            if (controls.getButton(Constants.kGroundPickupButton)) {
                state = HatchDeployStateEnum.GROUND;
            }
            break;
        case GROUND:
            dropMotor.set(ControlMode.MotionMagic, inchesToEncoderUnits(groundAngle));
            if (controls.getButton(Constants.kBumperButton)) {
                state = HatchDeployStateEnum.DEFENSE;
            }
            break;
        }

        //shoots both pistons from the solenoid 
        boolean ejectButton = controls.getButton(Constants.kHatchShootButton);
        HatchSolenoid.set(ejectButton);

    }

    public double inchesToEncoderUnits(double angle) {
        // to do: write this function
        return 0.0;
    }

}
