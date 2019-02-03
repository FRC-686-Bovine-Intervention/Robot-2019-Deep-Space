package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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
    public TalonSRX liftMotorMaster;
    public VictorSPX liftMotorSlave;
    public final int liftMotorMasterPort = 5;
    public final int liftSlaveMotorPort = 6;
    public final int cargoPort = 7;
    public final double rocketAngle = 0.65;
    public final double groundAngle = 0.25;
    public final double cargoAngle = 0.85;
    public final double defenseAngle = 0.95;
    public final double zeroingSpeed = -0.1;
    public final double IntakeSpeed = 1; 
    public final double OuttakeSpeed = -1; 
    public final double cargoBallStop = 0;

    public enum CargoBallEnum {
        START_POS, DEFENSE, CARGO, ROCKET, GROUND;
    }

    public CargoBallEnum state = CargoBallEnum.START_POS;

    public CargoBallIntake() {
        cargoIntakeMotor = new Talon(cargoPort);
        liftMotorMaster = new TalonSRX(liftMotorMasterPort);
        liftMotorSlave = new VictorSPX(liftSlaveMotorPort);
        limitSwitch = new DigitalInput(9);
        proximitySensor = new DigitalInput(11);
        state = CargoBallEnum.START_POS;
        liftMotorSlave.follow(liftMotorSlave);
    }

    public void run() {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        switch (state) {
        case START_POS:
            liftMotorMaster.set(ControlMode.PercentOutput, zeroingSpeed);
            if (limitSwitch.get()) {
                state = CargoBallEnum.DEFENSE;
            }
            break;
        case DEFENSE:
            liftMotorMaster.set(ControlMode.MotionMagic, inchesToEncoderUnits(defenseAngle));
            if (controls.getButton(Constants.kIntakeButton)) {
                state = CargoBallEnum.GROUND;
            }
            break;
        case GROUND:
            liftMotorMaster.set(ControlMode.MotionMagic, inchesToEncoderUnits(groundAngle));
            cargoIntakeMotor.set(IntakeSpeed);
            if (proximitySensor.get()) {
                cargoIntakeMotor.set(cargoBallStop);
                state = CargoBallEnum.CARGO;
            }
            break;
        case CARGO:
            liftMotorMaster.set(ControlMode.MotionMagic, inchesToEncoderUnits(cargoAngle));
            if (controls.getButton(Constants.kRocketButton)) {
                state = CargoBallEnum.ROCKET;
            }
            break;
        case ROCKET:
            liftMotorMaster.set(ControlMode.MotionMagic, inchesToEncoderUnits(rocketAngle));
            if (controls.getButton(Constants.kDefenseButton)) {
                state = CargoBallEnum.DEFENSE;
            }
        }
        
        if (controls.getButton(Constants.kOuttakeButton)) {
            cargoIntakeMotor.set(OuttakeSpeed);
        }
        else {
            cargoIntakeMotor.set(cargoBallStop);
        }
    }

    public double inchesToEncoderUnits(double angle) {
        // to do: write this function
        return 0.0;
    }

}