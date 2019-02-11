package  frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CargoBallIntake.CargoDeployPositionEnum;
import frc.robot.CargoBallIntake.CargoDeployStateEnum;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Drive;

public class Climber implements Loop
{
	// singleton class
    private static Climber instance = null;
    public static Climber getInstance() 
    { 
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }
    
    public CargoBallIntake arm = CargoBallIntake.getInstance();
    public ClimberCylinders cylinders = ClimberCylinders.getInstance();
    public VictorSPX climberDriveMotor = new VictorSPX(Constants.kClimberDriveMotorTalonId);
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();

    public enum ClimberStateEnum {ARMS_ON_PLATFORM, CLIMB, DRIVE_ONTO_PLATFORM, RETRACT, LAST_NUDGE, FINISHED};
    ClimberStateEnum climberState = ClimberStateEnum.ARMS_ON_PLATFORM;

    public final double kClimberMotorPercentOutput = 4/12;
    public final double kDriveMotorPercentOutput = 4/12;

    public double startRetractTime;
    public final double kRetractTimePeriod = 2.0;
    public final double kFinishTimePeriod = 0.5;

    public Climber()
    {
        cylinders.retract();

        climberDriveMotor.set(ControlMode.PercentOutput, 0.0);
		climberDriveMotor.setNeutralMode(NeutralMode.Brake);
		climberDriveMotor.setInverted(false);
		// current limiting not available on Victor SPX, so keep voltage low
    }
      

    @Override
    public void onStart()
    {
        cylinders.retract();
        climberDriveMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void onStop()
    {
        cylinders.off();
        climberDriveMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void onLoop()
    {
        // do nothing unless Climber mode is enabled
        if (arm.state == CargoDeployStateEnum.CLIMBING)
        {
            double currentTime = Timer.getFPGATimestamp();

            switch (climberState)
            {
            case ARMS_ON_PLATFORM:
                // slowly spin wheels forward
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                climberDriveMotor.set(ControlMode.PercentOutput, kClimberMotorPercentOutput);

                // set arm at height for platform
                arm.setPosition(CargoDeployPositionEnum.PLATFORM);

                if (buttonBoard.getButton(Constants.kClimbingExtendButton))
                {
                    climberState = ClimberStateEnum.CLIMB;
                }
                break;
                
            case CLIMB:
                // slowly spin wheels forward
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                climberDriveMotor.set(ControlMode.PercentOutput, kClimberMotorPercentOutput);

                cylinders.extend();

                // TODO: PID loop stuff

                if (arm.getForwardSoftLimit())
                {
                    climberState = ClimberStateEnum.DRIVE_ONTO_PLATFORM;
                }
                break;
                
            case DRIVE_ONTO_PLATFORM:
                // slowly spin wheels forward
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                climberDriveMotor.set(ControlMode.PercentOutput, kClimberMotorPercentOutput);

                arm.turnOffSoftLimits();                            // turn of soft limits so we can do a pushup
                arm.setPosition(CargoDeployPositionEnum.PUSHUP);    // push arm past soft limit to hard limit

                if (buttonBoard.getButton(Constants.kClimbingRetractButton))
                {
                    startRetractTime = currentTime;
                    climberState = ClimberStateEnum.RETRACT;
                }
                break;
                
            case RETRACT:
                // stop drive motors while cylinders are retracted
                Drive.getInstance().setOpenLoop(DriveCommand.COAST());
                climberDriveMotor.set(ControlMode.PercentOutput, 0.0);

                arm.setPosition(CargoDeployPositionEnum.RETRACTED);    // retract cargo arm so we can fit on platform

                cylinders.retract();                                    // retract cylinders

                if ((currentTime - startRetractTime) > kRetractTimePeriod)
                {
                    climberState = ClimberStateEnum.LAST_NUDGE;
                }
                break;
                
            case LAST_NUDGE:
                // drive forward a little bit more
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                
                if ((currentTime - startRetractTime) > (kRetractTimePeriod + kFinishTimePeriod))
                {
                    climberState = ClimberStateEnum.FINISHED;
                }
                break;

            case FINISHED:
                // done!
                Drive.getInstance().setOpenLoop(DriveCommand.COAST());
                break;
            }
        }
    }
}    
