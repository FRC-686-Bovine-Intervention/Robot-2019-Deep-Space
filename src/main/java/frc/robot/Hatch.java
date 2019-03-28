package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.joystick.SelectedJoystick;
import frc.robot.lib.util.FallingEdgeDetector;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.loops.Loop;

public class Hatch implements Loop
{
    public static Hatch mInstance = new Hatch();

    public static Hatch getInstance() {
        return mInstance;
    }

    public Solenoid hatchShootSolenoid;
    public Solenoid hatchExtendSolenoid;
    public RisingEdgeDetector hatchButtonRisingEdgeDetector = new RisingEdgeDetector();
    public FallingEdgeDetector hatchButtonFallingEdgeDetector = new FallingEdgeDetector();
    public RisingEdgeDetector ejectButtonRisingEdgeDetector = new RisingEdgeDetector();
    public FallingEdgeDetector ejectButtonFallingEdgeDetector = new FallingEdgeDetector();
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();

    //====================================================
    // Constants
    //====================================================
    public final int kSlotIdx = 0;
    public final int kPeakCurrentLimit = 30;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 20;
    
    public enum HatchStateEnum {
        INIT, ACQUIRE, HOLDHATCH, RELEASE, DEFENSE;
    }

    public HatchStateEnum state = HatchStateEnum.INIT;

    public Hatch() {
        
        hatchShootSolenoid = new Solenoid(0, Constants.kHatchEjectChannel);
        hatchExtendSolenoid = new Solenoid(1, Constants.kHatchEjectChannel);
        state = HatchStateEnum.INIT;       
    }

	@Override
	public void onStart() 
	{
            state = HatchStateEnum.INIT;
        
    }

    
	@Override
	public void onStop() 
	{
    }

    boolean drivingHatch;
    @Override
    public void onLoop() {
        drivingHatch = !SelectedJoystick.getInstance().getDrivingCargo();

        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        boolean dBtnIsPushed = buttonBoard.getButton(Constants.kDefenseButton);
        
        boolean hBtnIsPushed = controls.getButton(Constants.kHatchDeployButton) && drivingHatch;
        boolean hButtonPush = hatchButtonRisingEdgeDetector.update(hBtnIsPushed);
        boolean hButtonRelease = hatchButtonFallingEdgeDetector.update(hBtnIsPushed);

        boolean ejectButton = controls.getAxisAsButton(Constants.kHatchShootAxis) && drivingHatch;
        boolean ejectButtonPush = ejectButtonRisingEdgeDetector.update(ejectButton);
        boolean ejectButtonRelease = ejectButtonFallingEdgeDetector.update(ejectButton);
        
        switch (state) {
        case INIT:
            
            hatchShootSolenoid.set(false);
            hatchExtendSolenoid.set(true);
            if (hButtonPush){
            state = HatchStateEnum.ACQUIRE; 
            }
            if(ejectButtonPush){
                state = HatchStateEnum.HOLDHATCH;
            }
            if (dBtnIsPushed){
                state = HatchStateEnum.DEFENSE;
            }
        
            break;


        case ACQUIRE:
            hatchShootSolenoid.set(true);
            hatchExtendSolenoid.set(false);
            if(hButtonRelease){
                state = HatchStateEnum.HOLDHATCH;
            }
            else if (ejectButtonPush){
                state = HatchStateEnum.HOLDHATCH;
            }
            if (dBtnIsPushed){
                state = HatchStateEnum.DEFENSE;
            }
        break;

        case HOLDHATCH:
            hatchShootSolenoid.set(true);
            hatchExtendSolenoid.set(true);
            if(ejectButtonRelease){
                state = HatchStateEnum.RELEASE;
            }
            if (hButtonPush){
                state = HatchStateEnum.ACQUIRE;
            }
            if (dBtnIsPushed){
                state = HatchStateEnum.DEFENSE;
            }
            break;

        case RELEASE:
            hatchShootSolenoid.set(false);
            hatchExtendSolenoid.set(false);
            if(ejectButtonPush){
                state = HatchStateEnum.HOLDHATCH;
            }
            if (hButtonPush){
                state = HatchStateEnum.ACQUIRE;
            }
            if (dBtnIsPushed){
                state = HatchStateEnum.DEFENSE;
            }
            break;

            case DEFENSE:
                hatchShootSolenoid.set(false);
                hatchExtendSolenoid.set(false);
                if(ejectButtonPush){
                    state = HatchStateEnum.HOLDHATCH;
                }
                if (hButtonPush){
                    state = HatchStateEnum.ACQUIRE;
                }
             break;
        }
    }
    public void open() {
        hatchShootSolenoid.set(true);
    }
    public void close() {
        hatchShootSolenoid.set(false);
    }
    public void extended() {
        hatchExtendSolenoid.set(true);
    }
    public void retracted() {
        hatchExtendSolenoid.set(false);
    }

    public void setState(HatchStateEnum _newState)
    {
        state = _newState;
    }
}
