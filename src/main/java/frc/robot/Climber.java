package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CargoIntake.CargoDeployPositionEnum;
import frc.robot.CargoIntake.CargoDeployStateEnum;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Util;
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
    
    public VictorSPX climberDriveMotor;
    public CargoIntake arm = CargoIntake.getInstance();
    public ClimberCylinders cylinders = ClimberCylinders.getInstance();
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();
    public double pushUpThresholdLevel2 = -5;
    public double pushUpThresholdLevel3 = 5;
    public double level2ChangeAngleStartTime;
    
    public enum ClimberStateEnum {INIT, LEVEL3_ARMS_ON_PLATFORM, LEVEL2_ARMS_ON_PLATFORM, LEVEL3_CLIMB, LEVEL2_CLIMB,
                                  LEVEL3_DRIVE_ONTO_PLATFORM, LEVEL2_DRIVE_ONTO_PLATFORM,
                                  RETRACT_CYLINDERS, LAST_NUDGE, FINISHED};
    static ClimberStateEnum climberState = ClimberStateEnum.LEVEL3_ARMS_ON_PLATFORM;
    public int platformLevel = 3;

    public final double kDriveMotorPercentOutput = 0.3;
    public final double kClimberMotorWhenExtendingPercentOutput = 0.2; 
    public final double kClimberMotorAtTopPercentOutput = 0.75; //0.5

    public double startRetractTime;
    public final double kRetractTimePeriod = 2.0;
    public final double kFinishTimePeriod = 0.5;

    // gyro to get tilt of robot
    public Pigeon pigeon = (Pigeon)Pigeon.getInstance();
    public double tiltAngleDeg = 0.0;

    // PID Loop
    double Kp = 0.01;
    double Kd = 0.0;
    double Ki = 0.0;

    double error = 0.0;
    double dError = 0.0;
    double iError = 0.0;
    double lastError = 0.0;

    double pidOutput = 0.0;

    double level2LastTimer;
    double level2InitialWait = 2.25;
    double level2ToggleWait = 0.25;
    int toggleCount = 0;
    int level2NumToggles = 16;

    public Climber()
    {
        climberDriveMotor = new VictorSPX(Constants.kClimberDriveMotorTalonId);
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
        climberState = ClimberStateEnum.LEVEL3_ARMS_ON_PLATFORM;
        climberDriveMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void onStop()
    {
        cylinders.off();
        climberDriveMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    public static void startOver()
    {
        climberState = ClimberStateEnum.INIT;
    }

    @Override
    public void onLoop()
    {
        tiltAngleDeg = pigeon.getPitchDeg();
        
        if (arm.state == CargoDeployStateEnum.OPERATIONAL)
        {
            cylinders.retract();
        }

        // do nothing unless Climber mode is enabled
        if (arm.state == CargoDeployStateEnum.CLIMBING)
        {
            double currentTime = Timer.getFPGATimestamp();
            
            switch (climberState)
            {
            case INIT:
                climberState = ClimberStateEnum.LEVEL3_ARMS_ON_PLATFORM;
                cylinders.retract();//retract cylinders for climber abort
                break;

            case LEVEL3_ARMS_ON_PLATFORM:
                platformLevel = 3;
                // slowly spin wheels forward to square with platform
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                
                // set arm at height for platform
                arm.setTarget(CargoDeployPositionEnum.HAB_LEVEL3);
                
                if (CargoIntake.climbingStartEdgeDetector.get())
                {
                    // if climb button is pressed a 2nd time, move on to Level2
                    climberState = ClimberStateEnum.LEVEL2_ARMS_ON_PLATFORM;
                }
                if (buttonBoard.getButton(Constants.kClimbingExtendButton))
                {
                    climberState = ClimberStateEnum.LEVEL3_CLIMB;
                }
                break;
                
            case LEVEL2_ARMS_ON_PLATFORM:
                platformLevel = 2;
                
                // set arm at height for platform
                arm.setTarget(CargoDeployPositionEnum.HAB_LEVEL2);
                
                if (CargoIntake.climbingStartEdgeDetector.get())
                {
                    // if button is pressed a 3rd time, go back to retracted state
                    arm.setState(CargoDeployStateEnum.OPERATIONAL);
                    arm.setTarget(CargoDeployPositionEnum.RETRACTED);
                    startOver();   
                }
                if (buttonBoard.getButton(Constants.kClimbingExtendButton))
                {
                    climberState = ClimberStateEnum.LEVEL2_CLIMB;
                    level2LastTimer = Timer.getFPGATimestamp();
                    toggleCount = 0;
                }
                break;
                
            case LEVEL3_CLIMB:
                cylinders.extend();
                
                // PID loop
                error = -tiltAngleDeg;
                dError = (error - lastError) / Constants.kLoopDt;
                iError += (error * Constants.kLoopDt);
                lastError = error;
                pidOutput = Kp * error + Kd * dError + Ki * iError;
                pidOutput = Util.limit(pidOutput, 0, 1);
                arm.setPercentOutput(pidOutput);
                
                // once arms are down, move on
                if (arm.getArmAngleDeg() <= pushUpThresholdLevel3)
                {
                    arm.setPercentOutput(0.0);
                    climberState = ClimberStateEnum.LEVEL3_DRIVE_ONTO_PLATFORM;
                }
                break;
                
            case LEVEL2_CLIMB:
                cylinders.extend();
                    
                // PID loop
                error = -tiltAngleDeg;
                dError = (error - lastError) / Constants.kLoopDt;
                iError += (error * Constants.kLoopDt);
                lastError = error;
                pidOutput = Kp * error + Kd * dError + Ki * iError;
                pidOutput = Util.limit(pidOutput, 0, 1);

                arm.setPercentOutput(pidOutput);
                
                // once arms are down, move on
                if (arm.getArmAngleDeg() <= pushUpThresholdLevel2)
                {
                    // arm.setPercentOutput(0.0);
                    climberState = ClimberStateEnum.LEVEL2_DRIVE_ONTO_PLATFORM;
                    level2ChangeAngleStartTime = Timer.getFPGATimestamp();
                }
                break;

                    
            case LEVEL2_DRIVE_ONTO_PLATFORM:
                // slowly spin wheels forward
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                climberDriveMotor.set(ControlMode.PercentOutput, kClimberMotorAtTopPercentOutput);
                
                arm.turnOffSoftLimits(); // turn of soft limits so we can do a pushup
                arm.setTarget(CargoDeployPositionEnum.LEVEL2_APPROACH);    // push arm until level with frame
                
                
                // the piston dance
                if (toggleCount == 0)
                {
                    cylinders.extend();
                    if (Timer.getFPGATimestamp() - level2LastTimer >= level2InitialWait)
                    {
                        level2LastTimer = Timer.getFPGATimestamp();
                        toggleCount++;
                    }
                }
                else
                {
                    if (toggleCount % 2 == 1)
                    {
                        // toggleCount is odd
                        cylinders.retract();
                        if (Timer.getFPGATimestamp() - level2LastTimer >= level2ToggleWait)
                        {
                            level2LastTimer = Timer.getFPGATimestamp();
                            toggleCount++;
                        }
                    }
                    else
                    {
                        // toggleCount is even
                        cylinders.extend();
                        if (Timer.getFPGATimestamp() - level2LastTimer >= level2ToggleWait)
                        {
                            level2LastTimer = Timer.getFPGATimestamp();
                            toggleCount++;
                        }
                    }
                    if (toggleCount >= level2NumToggles)
                    {
                        // hopefully we're on the platform by now
                        startRetractTime = currentTime;
                        climberState = ClimberStateEnum.RETRACT_CYLINDERS;
                    }
                }   


                break;

            case LEVEL3_DRIVE_ONTO_PLATFORM:
                // slowly spin wheels forward
                Drive.getInstance().setOpenLoop(new DriveCommand(kDriveMotorPercentOutput, kDriveMotorPercentOutput));
                climberDriveMotor.set(ControlMode.PercentOutput, kClimberMotorAtTopPercentOutput);
                
                arm.turnOffSoftLimits(); // turn of soft limits so we can do a pushup
                arm.setTarget(CargoDeployPositionEnum.PUSHUP);    // push arm past soft limit to hard limit
                
                if (buttonBoard.getButton(Constants.kClimbingRetractButton))
                {
                    startRetractTime = currentTime;
                    climberState = ClimberStateEnum.RETRACT_CYLINDERS;
                }
                break;

            case RETRACT_CYLINDERS:
                // stop drive motors while cylinders are retracted
                Drive.getInstance().setOpenLoop(DriveCommand.COAST());
                climberDriveMotor.set(ControlMode.PercentOutput, 0.0);
                 
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
                    arm.setState(CargoDeployStateEnum.OPERATIONAL);
                    startOver();                       
                }
                break;

            case FINISHED:
                // nothing to do
                break;                
            }
        }
    }
    
    public boolean shouldBlink()
    {
        // blink if at beginning of climb sequence to let drivers know climber is active
        // blink when finished to celebrate
        return ((arm.state == CargoDeployStateEnum.CLIMBING) && (climberState == ClimberStateEnum.LEVEL3_ARMS_ON_PLATFORM)) || 
               ((arm.state == CargoDeployStateEnum.CLIMBING) && (climberState == ClimberStateEnum.LEVEL2_ARMS_ON_PLATFORM) || 
               (climberState == ClimberStateEnum.FINISHED)); 
    }

	private final DataLogger logger = new DataLogger()
	{
        @Override
		public void log()
		{
            put("Climber/state", climberState.toString());
            put("Climber/tiltAngleDeg", tiltAngleDeg);
            put("Climber/error", error);
            put("Climber/derror", dError);
            put("Climber/iError", iError);
            put("Climber/pidOutput", pidOutput);

            boolean climbing = (arm.state == CargoDeployStateEnum.CLIMBING);
            boolean finished = climbing && (climberState == ClimberStateEnum.FINISHED);
            boolean retract =  climbing && (climberState == ClimberStateEnum.RETRACT_CYLINDERS);
            boolean l3Drive = climbing && (climberState == ClimberStateEnum.LEVEL3_DRIVE_ONTO_PLATFORM);
            boolean l3Climb = climbing && (climberState == ClimberStateEnum.LEVEL3_CLIMB);
            boolean l3Arms  = climbing && (climberState == ClimberStateEnum.LEVEL3_ARMS_ON_PLATFORM);
            boolean l2Drive = climbing && (climberState == ClimberStateEnum.LEVEL2_DRIVE_ONTO_PLATFORM);
            boolean l2Climb = climbing && (climberState == ClimberStateEnum.LEVEL2_CLIMB);
            boolean l2Arms  = climbing && (climberState == ClimberStateEnum.LEVEL2_ARMS_ON_PLATFORM);

            put("Climber/climbMode", climbing);
            put("Climber/l3Arms",   l3Arms);
            put("Climber/l3Climb",  l3Climb);
            put("Climber/l3Drive",  l3Drive);
            put("Climber/l2Arms",   l2Arms);
            put("Climber/l2Climb",  l2Climb);
            put("Climber/l2Drive",  l2Drive);
            put("Climber/retract",  retract);
            put("Climber/finished", finished);
            put("Climber/CylinderDirection", cylinders.direction.toString());

		}
	};
    
	public DataLogger getLogger()
	{
		return logger;
	}        
}    
