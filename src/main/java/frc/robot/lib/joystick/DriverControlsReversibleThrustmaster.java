package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;



public class DriverControlsReversibleThrustmaster extends ReversibleDriverControlsBase
{
	// singleton class
    private static DriverControlsReversibleThrustmaster instance = null;
    public static DriverControlsReversibleThrustmaster getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsReversibleThrustmaster();
        }
        return instance;
    }
    
    
    // Joystick Port Constants
    public static int kLeftStickPort = 0;
    public static int kRightStickPort = 1;
    public static int kButtonBoardPort = 2;

    public static JoystickBase lStick;
    public static JoystickBase rStick;
    public static JoystickBase buttonBoard;

    public static ReversibleSteeringBase steeringControls;

    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonA;
    public static int kDefenseButton =              ButtonBoard.kButtonRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonSR;

    public DriverControlsReversibleThrustmaster() 
    {
        lStick = new Thrustmaster(kLeftStickPort);
        rStick = new Thrustmaster(kRightStickPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmReversibleArcadeDriveSteering(lStick, rStick, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand() 
    {
        return steeringControls.getDriveCommand(); 
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        switch (_control)
        {
            case VISION_ASSIST:                 return lStick.getButton(Thrustmaster.kLeftThumbButton);
            case HATCH_DEPLOY:                  return rStick.getButton(Thrustmaster.kBottomThumbButton);
            case HATCH_SHOOT:                   return rStick.getButton(Thrustmaster.kTriggerButton);
            case CARGO_INTAKE:                  return lStick.getButton(Thrustmaster.kBottomThumbButton);
            case CARGO_OUTTAKE:                 return lStick.getButton(Thrustmaster.kTriggerButton);
            case CARGO_INTAKE_DEPOT_HEIGHT:     return rStick.getButton(Thrustmaster.kRightThumbButton);
            case CARGO_INTAKE_ROCKET_HEIGHT:    return buttonBoard.getButton(kCargoIntakeRocketButton);
            case CARGO_INTAKE_CARGO_HEIGHT:     return buttonBoard.getButton(kCargoIntakeCargoShipButton);
            case DEFENSE:                       return buttonBoard.getButton(kDefenseButton);
            case CLIMB_PREPARE:                 return buttonBoard.getButton(kClimbingStartButton);
            case CLIMB_EXTEND:                  return buttonBoard.getButton(kClimbingExtendButton);
            case CLIMB_RETRACT:                 return buttonBoard.getButton(kClimbingRetractButton);
            case EMERGENCY_ZEROING:             return buttonBoard.getButton(kEmergencyZeroingAxis);
            case QUICK_TURN:                    return false;
            default:                            return false;
        }
    }

    public boolean getDrivingCargo()
    {
        return steeringControls.usingLeftStick();
    }

    public boolean joystickActive()
    {
        return steeringControls.joystickActive();
    }



    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (lStick != null)             { lStick.getLogger().log(); }
            if (rStick != null)             { rStick.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };    

}
