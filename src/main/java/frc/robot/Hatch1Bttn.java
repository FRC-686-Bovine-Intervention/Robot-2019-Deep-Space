package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.joystick.SelectedJoystick;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.FallingEdgeDetector;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.loops.Loop;

public class Hatch1Bttn implements Loop {
    public static Hatch1Bttn mInstance = new Hatch1Bttn();

    public static Hatch1Bttn getInstance() {
        return mInstance;
    }

    public Solenoid hatchGrabSolenoid;
    public Solenoid hatchExtendSolenoid;
    public DigitalInput hatchDetectSensor;
    private double mStartTime;
    private double mTimeToWait = 0.25;
    public RisingEdgeDetector hatchButtonRisingEdgeDetector = new RisingEdgeDetector();
    public FallingEdgeDetector hatchButtonFallingEdgeDetector = new FallingEdgeDetector();
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();

    public enum HatchStateEnum {
        INIT, TO_EXTEND, EXTEND, GRAB_RELEASE, RETRACT, DEFENSE;
    }

    public HatchStateEnum state = HatchStateEnum.INIT;

    public Hatch1Bttn() {

        hatchGrabSolenoid = new Solenoid(0, Constants.kHatchGrabChannel);
        hatchExtendSolenoid = new Solenoid(0, Constants.kHatchExtendChannel);
        hatchDetectSensor = new DigitalInput(Constants.kHatchDetectSensorPort);
        state = HatchStateEnum.INIT;
    }

    @Override
    public void onStart() {
        state = HatchStateEnum.INIT;
        open();
        retract();
    }

    @Override
    public void onStop() {
    }

    boolean dBtnIsPushed = false;

    boolean hatchButton = false;
    boolean hatchButtonPush = false;
    boolean hatchButtonRelease = false;

    boolean hatchDetected = false;

    boolean drivingHatch;

    @Override
    public void onLoop() {
        drivingHatch = !SelectedJoystick.getInstance().getDrivingCargo();

        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        dBtnIsPushed = buttonBoard.getButton(Constants.kDefenseButton);

        hatchButton = controls.getAxisAsButton(Constants.kHatchShootAxis) && drivingHatch;
        hatchButtonPush = hatchButtonRisingEdgeDetector.update(hatchButton);
        hatchButtonRelease = hatchButtonFallingEdgeDetector.update(hatchButton);

        if (hatchButtonPush && state == HatchStateEnum.RETRACT) {
            state = HatchStateEnum.TO_EXTEND;
        }
        if (dBtnIsPushed) {
            state = HatchStateEnum.DEFENSE;
        }

        switch (state) {
        case INIT:
            open();
            retract();
            state = HatchStateEnum.RETRACT;
            break;

        case TO_EXTEND:
            extend();
            hatchDetected = hatchDetect();
            if (hatchDetected) {
                open();
            } else {
                close();
            }
            state = HatchStateEnum.EXTEND;
            break;

            case EXTEND:
            if (hatchButtonPush) {
                mStartTime = Timer.getFPGATimestamp();
                state = HatchStateEnum.GRAB_RELEASE;
            }
            break;

        case GRAB_RELEASE:
            extend();
            if (hatchDetected) {
                close();
            } else {
                open();
            }
            if (Timer.getFPGATimestamp() - mStartTime >= mTimeToWait) {
                state = HatchStateEnum.RETRACT;
            }
            break;

        case RETRACT:
            retract();
            break;

        case DEFENSE:
            close();
            retract();
            break;
        }
    }

    public void open() {
        hatchGrabSolenoid.set(true);
    }

    public void close() {
        hatchGrabSolenoid.set(false);
    }

    public void extend() {
        hatchExtendSolenoid.set(true);
    }

    public void retract() {
        hatchExtendSolenoid.set(false);
    }

    public boolean hatchDetect() {
        return !hatchDetectSensor.get();
    }

    public void setState(HatchStateEnum _newState) {
        state = _newState;
    }

    private final DataLogger logger = new DataLogger() {
        @Override
        public void log() {
            put("Hatch/state", state.toString());
            put("Hatch/drivingHatch", drivingHatch);
            put("Hatch/dBtnIsPushed", dBtnIsPushed);
            put("Hatch/grabBtnIsPushed", hatchButton);
            put("Hatch/grabButtonPush", hatchButtonPush);
            put("Hatch/grabButtonRelease", hatchButtonRelease);
        }
    };

    public DataLogger getLogger() {
        return logger;
    }

}
