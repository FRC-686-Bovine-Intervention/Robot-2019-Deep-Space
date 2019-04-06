package frc.robot;

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

public class Hatch implements Loop {
    public static Hatch mInstance = new Hatch();

    public static Hatch getInstance() {
        return mInstance;
    }

    public Solenoid hatchGrabSolenoid;
    public Solenoid hatchExtendSolenoid;
    private double mStartTime;
    private double mTimeToWait = 0.25;
    public RisingEdgeDetector grabButtonRisingEdgeDetector = new RisingEdgeDetector();
    public FallingEdgeDetector grabButtonFallingEdgeDetector = new FallingEdgeDetector();
    public RisingEdgeDetector extendButtonRisingEdgeDetector = new RisingEdgeDetector();
    public FallingEdgeDetector extendButtonFallingEdgeDetector = new FallingEdgeDetector();
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();


    public enum HatchStateEnum {
        INIT, ACQUIRE, ACQUIRE_DELAY, HOLDHATCH, RELEASE, RELEASE_DELAY, DEFENSE;
    }

    public HatchStateEnum state = HatchStateEnum.INIT;

    public Hatch() {
        
        hatchGrabSolenoid   = new Solenoid(0, Constants.kHatchGrabChannel);
        hatchExtendSolenoid = new Solenoid(0, Constants.kHatchExtendChannel);
        state = HatchStateEnum.INIT;       
    }

	@Override
	public void onStart() 
	{
        state = HatchStateEnum.INIT;
        open();
        retract();
    }

    
	@Override
	public void onStop() 
	{
    }


    boolean dBtnIsPushed = false;
        
    boolean grabBtnIsPushed =false;
    boolean grabButtonPush = false;
    boolean grabButtonRelease = false;

    boolean extendButton = false ;
    boolean extendButtonPush = false;
    boolean extendButtonRelease = false;

    boolean drivingHatch;
    @Override
    public void onLoop() {
        drivingHatch = !SelectedJoystick.getInstance().getDrivingCargo();

        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
         dBtnIsPushed = buttonBoard.getButton(Constants.kDefenseButton);
        
         grabBtnIsPushed = controls.getButton(Constants.kHatchDeployButton) && drivingHatch;
         grabButtonPush = grabButtonRisingEdgeDetector.update(grabBtnIsPushed);
         grabButtonRelease = grabButtonFallingEdgeDetector.update(grabBtnIsPushed);

         extendButton = controls.getAxisAsButton(Constants.kHatchShootAxis) && drivingHatch;
         extendButtonPush = extendButtonRisingEdgeDetector.update(extendButton);
         extendButtonRelease = extendButtonFallingEdgeDetector.update(extendButton);
        
        if (grabButtonPush) {state = HatchStateEnum.ACQUIRE;}
        if (extendButtonPush) {state = HatchStateEnum.HOLDHATCH;}
        if (dBtnIsPushed) {state = HatchStateEnum.DEFENSE;}

        switch (state) {
        case INIT:
            // don't do anything in this state -- it messes up autonomous
            break;

        case ACQUIRE:
            close();
            extend();
            if(grabButtonRelease){
                mStartTime = Timer.getFPGATimestamp();
                state = HatchStateEnum.ACQUIRE_DELAY;
            }
        break;

        case ACQUIRE_DELAY: {
            open();
            if ( Timer.getFPGATimestamp() - mStartTime >= mTimeToWait)    
            {
                state = HatchStateEnum.HOLDHATCH;
            }
        }
        break;

        case HOLDHATCH:
            open();
            extend();
            if(extendButtonRelease){
                mStartTime = Timer.getFPGATimestamp();
                state = HatchStateEnum.RELEASE_DELAY;
            }
            break;

            case RELEASE_DELAY: {
                close();
                if ( Timer.getFPGATimestamp() - mStartTime >= mTimeToWait)    
                {
                    state = HatchStateEnum.RELEASE;
                }
            }
            break;

        case RELEASE:    
            retract();
            break;

            case DEFENSE:
               close();
                retract();
             break;
        }
    }

    public void open() {
        hatchGrabSolenoid.set(false);
    }
    public void close() {
        hatchGrabSolenoid.set(true);
    }
    public void extend() {
        hatchExtendSolenoid.set(true);
    }
    public void retract() {
        hatchExtendSolenoid.set(false);
    }

    public void setState(HatchStateEnum _newState)
    {
        state = _newState;
    }
        
private final DataLogger logger = new DataLogger()
{
    @Override
    public void log()
    {
        put("Hatch/state", state.toString());
        put("Hatch/drivingHatch", drivingHatch);
        put("Hatch/dBtnIsPushed", dBtnIsPushed);
        put("Hatch/grabBtnIsPushed", grabBtnIsPushed);
        put("Hatch/grabButtonPush", grabButtonPush);
        put("Hatch/grabButtonRelease", grabButtonRelease);
        put("Hatch/extendButton", extendButton);
        put("Hatch/extendButtonPush", extendButtonPush);
        put("Hatch/extendButtonRelease", extendButtonRelease);
    }
};

public DataLogger getLogger()
{
    return logger;
}    

}
