package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class CargoBallIntake {

    public static CargoBallIntake mInstance = new CargoBallIntake();

    public static CargoBallIntake getInstance() {
        return mInstance;
    }

    public DigitalInput limitSwitch;
    public DigitalInput proximitySensor;
    public Talon cargoIntakeMotor;
    public TalonSRX liftMechanism;
    public final int liftPort = 1;
    public final int cargoPort = 2;
    public final double rocketAngle = 0.65;
    public final double groundAngle = 0.25;
    public final double cargoAngle = 0.85;
    public final double defenseAngle = 0.95;
    public final double zeroingSpeed = -0.1;
    public final double cargoBallSpeed = 1;
    public final double cargoBallStop = 0;

    public enum CargoBallEnum {
        START_POS, DEFENSE, CARGO, ROCKET, GROUND;
    }

    public CargoBallEnum state = CargoBallEnum.START_POS;

    public CargoBallIntake() {
        cargoIntakeMotor = new Talon(cargoPort);
        liftMechanism = new TalonSRX(liftPort);
        limitSwitch = new DigitalInput(5);
        state = CargoBallEnum.START_POS;
    }

    public void run() {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        switch (state) {
        case START_POS:
            liftMechanism.set(ControlMode.PercentOutput, zeroingSpeed);
            if (limitSwitch.get()) {
                state = CargoBallEnum.DEFENSE;
            }
            break;
        case DEFENSE:
            liftMechanism.set(ControlMode.MotionMagic, inchesToEncoderUnits(defenseAngle));
            if (controls.getButton(Constants.kIntakeButton)) {
                state = CargoBallEnum.GROUND;
            }
            break;
        case GROUND:
            liftMechanism.set(ControlMode.MotionMagic, inchesToEncoderUnits(groundAngle));
            cargoIntakeMotor.set(cargoBallSpeed);
            if (proximitySensor.get()) {
                state = CargoBallEnum.CARGO;
            }
            break;
        case CARGO:
            cargoIntakeMotor.set(cargoBallStop);
            liftMechanism.set(ControlMode.MotionMagic, inchesToEncoderUnits(cargoAngle));
            if (controls.getButton(Constants.kRocketButton)) {
                state = CargoBallEnum.ROCKET;
            }
            break;
        case ROCKET:
            liftMechanism.set(ControlMode.MotionMagic, inchesToEncoderUnits(rocketAngle));
            if (controls.getButton(Constants.kDefenseButton)) {
                state = CargoBallEnum.DEFENSE;
            }
        }
        
        if (controls.getButton(Constants.kOuttakeButton)) {
            cargoIntakeMotor.set(cargoBallSpeed);
        }
    }

    public double inchesToEncoderUnits(double angle) {
        // to do: write this function
        return 0.0;
    }

}