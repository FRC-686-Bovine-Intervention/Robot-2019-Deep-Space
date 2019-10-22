package frc.robot.lib.joystick;

import frc.robot.CargoIntake;
import frc.robot.CargoIntake.CargoDeployPositionEnum;
import frc.robot.SmartDashboardInteractions;
import frc.robot.command_status.DriveCommand;


public class DriverControls
{
    private static DriverControls mInstance = new DriverControls();

    public static DriverControls getInstance() 
    {
        return mInstance;
    }

    public enum SchemeEnum 
    {
        XBOX("XBox"),
        THRUSTMASTER("ThrustMaster"),
        MICHAEL("Michael"),
        TYLER("Tyler"),				
        BEN("Ben");
 
    	public final String name;
    	
        SchemeEnum(String name) {
    		this.name= name;
    	}
    }

    // Joystick Port Constants
    public static int kThrustMasterLStickPort = 0;
    public static int kThrustMasterRStickPort = 1;
    public static int kButtonBoardPort = 2;

    public static JoystickBase lStick;
    public static JoystickBase rStick;
    public static JoystickBase buttonBoard;


    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonA;
    public static int kDefenseButton =              ButtonBoard.kButtonRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonSR;

    public SchemeEnum controlScheme = SchemeEnum.THRUSTMASTER;

    public DriverControls() 
    {
        lStick = new Thrustmaster(kThrustMasterLStickPort);
        rStick = new Thrustmaster(kThrustMasterRStickPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);
    }

    public void updateDriverControlScheme() 
    {
        controlScheme = SmartDashboardInteractions.getInstance().getDriverControlsScheme();
    }

    public DriveCommand getDriveCommand() 
    {
        switch (controlScheme)  // control scheme selected in SmartDashboard
        {
            case XBOX:             
            case THRUSTMASTER:  
            case MICHAEL:       
            case TYLER:         
            case BEN:           
            default:            
                return TmReversibleArcadeDriveJoystick.getDriveCommand(lStick, rStick); 
        }
    }

    public JoystickBase.ThrottleTurn getThrottleTurn() 
    {
        switch (controlScheme)  // control scheme selected in SmartDashboard
        {
            case XBOX:             
            case THRUSTMASTER:  
            case MICHAEL:       
            case TYLER:         
            case BEN:           
            default:            
                return TmReversibleArcadeDriveJoystick.getThrottleTurn(lStick, rStick); 
        }
    }    

 
    public boolean getDrivingCargo()       { return controls.usingLeftStick(); }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        switch (controlScheme)  // control scheme selected in SmartDashboard
        {
            case XBOX:
            default:
                switch (_control)
                {
                    case VISION_ASSIST:                 return lStick.getButton(XboxConstants.kButtonA);
                    case HATCH_DEPLOY:                  return lStick.getButton(XboxConstants.kButtonRB);
                    case HATCH_SHOOT:                   return lStick.getButton(XboxConstants.kRTriggerAxis);
                    case CARGO_INTAKE:                  return lStick.getButton(XboxConstants.kButtonLB);
                    case CARGO_OUTTAKE:                 return lStick.getButton(XboxConstants.kLTriggerAxis);
                    case CARGO_INTAKE_DEPOT_HEIGHT:     return lStick.getButton(XboxConstants.kButtonX);
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
                // end case XBOX
    
                case THRUSTMASTER:
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
                // end case THRUSTMASTER

                case MICHAEL:
                switch (_control)
                {
                    case VISION_ASSIST:                 return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kLeftThumbButton);
                    case HATCH_DEPLOY:                  return controls.getPOV(ThrustMasterConstants.kRightStick) == 45 || controls.getPOV(ThrustMasterConstants.kRightStick) == 315 || controls.getPOV(ThrustMasterConstants.kRightStick) == 0;
                    case HATCH_SHOOT:                   return controls.getPOV(ThrustMasterConstants.kRightStick) <= 225 && controls.getPOV(ThrustMasterConstants.kRightStick) >= 135;
                    case CARGO_INTAKE:                  return riseEdgeDetect && CargoIntake.getInstance().getTarget() != CargoDeployPositionEnum.GROUND;
                    case CARGO_OUTTAKE:                 return controls.getPOV(ThrustMasterConstants.kLeftStick) == 0;
                    case CARGO_INTAKE_DEPOT_HEIGHT:     return oldEdgeDetect && CargoIntake.getInstance().getTarget() == CargoDeployPositionEnum.GROUND;
                    case CARGO_INTAKE_ROCKET_HEIGHT:    return controls.getButton(ThrustMasterConstants.kLeftStick, ThrustMasterConstants.kRightThumbButton);
                    case CARGO_INTAKE_CARGO_HEIGHT:     return controls.getButton(ThrustMasterConstants.kLeftStick, ThrustMasterConstants.kBottomThumbButton);
                    case DEFENSE:                       return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kTriggerButton);
                    case CLIMB_PREPARE:                 return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kTopButton4);
                    case CLIMB_EXTEND:                  return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kTopButton5);
                    case CLIMB_RETRACT:                 return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kTopButton6);
                    case EMERGENCY_ZEROING:             return buttonBoard.getButton(kEmergencyZeroingAxis);
                    case QUICK_TURN:                    return false;
                    default:                            return false;
                }
                // end case MICHAEL

                case TYLER:
                switch (_control)
                {
                    case VISION_ASSIST:                 return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kLeftThumbButton) || controls.getButton(ThrustMasterConstants.kRightStick,  ThrustMasterConstants.kLeftThumbButton);
                    case HATCH_DEPLOY:                  return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kBottomThumbButton);
                    case HATCH_SHOOT:                   return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kTriggerButton);
                    case CARGO_INTAKE:                  return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kBottomThumbButton);
                    case CARGO_OUTTAKE:                 return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kTriggerButton);
                    case CARGO_INTAKE_DEPOT_HEIGHT:     return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kRightThumbButton);
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
                // end case TYLER

                case BEN:   return false;
                    // TODO: replace above 'return false' with modified copy of above
                // end case BEN
            }
    }

}
