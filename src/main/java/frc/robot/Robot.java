/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.TimeZone;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.auto.modes.DriveStraightMode;
import frc.robot.auto.modes.HatchAuto;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.GoalStates;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.joystick.SelectedJoystick;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.ControlsReverse;
import frc.robot.lib.util.CrashTracker;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Pose;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.GoalStateLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.vision.VisionDriveAssistant;
import frc.robot.vision.VisionLoop;
import frc.robot.vision.VisionTargetList;

public class Robot extends TimedRobot {

	PowerDistributionPanel pdp = new PowerDistributionPanel();

	SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();
	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
	SelectedJoystick selectedJoystick = SelectedJoystick.getInstance();

	RobotState robotState = RobotState.getInstance();
	Drive drive = Drive.getInstance();
	VisionTargetList visionTargetList = VisionTargetList.getInstance();
	VisionDriveAssistant visionDriveAssistant = VisionDriveAssistant.getInstance();

	Superstructure superStructure = Superstructure.getInstance();

	AutoModeExecuter autoModeExecuter = null;

	LoopController loopController;

	DataLogController robotLogger;

	Limelight cargoCamera = Limelight.getCargoInstance();
	Limelight hatchCamera = Limelight.getHatchInstance();

	HatchDeploy hatchDeploy;
	ControlsReverse controlsReverse = ControlsReverse.getInstance();

	enum OperationalMode 
    {
    	DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);
    	
    	private int val;
    	
    	private OperationalMode (int val) {this.val = val;}
    	public int getVal() {return val;}
    } 
    
    OperationalMode operationalMode = OperationalMode.DISABLED;
    
    public Robot() {
    	CrashTracker.logRobotConstruction();
    }
    
    
	@Override
	public void robotInit() {
		try
    	{
    		CrashTracker.logRobotInit();

			LiveWindow.disableTelemetry(pdp);	// stops CAN error

			hatchDeploy = HatchDeploy.getInstance();
    		
    		loopController = new LoopController();
    		loopController.register(drive.getVelocityPIDLoop());
    		loopController.register(DriveLoop.getInstance());
       		loopController.register(RobotStateLoop.getInstance());
    		loopController.register(VisionLoop.getInstance());
			loopController.register(GoalStateLoop.getInstance());
			loopController.register(CargoIntake.getInstance());
			loopController.register(HatchDeploy.getInstance());
			loopController.register(Climber.getInstance());

			selectedJoystick.update();
    		
    		// set datalogger and time info
    		TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));
    		
    		robotLogger = DataLogController.getRobotLogController();
    		robotLogger.register(this.getLogger());
			robotLogger.register(Drive.getInstance().getLogger());
			robotLogger.register(drive.getCommand().getLogger());
			robotLogger.register(DriveState.getInstance().getLogger());
			robotLogger.register(RobotState.getInstance().getLogger());
			robotLogger.register(VisionLoop.getInstance().getLogger());
			robotLogger.register(VisionTargetList.getInstance().getLogger());
			robotLogger.register(GoalStateLoop.getInstance().getLogger());
			robotLogger.register(GoalStateLoop.getInstance().getGoalTracker().getLogger());
			robotLogger.register(GoalStates.getInstance().getLogger());
			robotLogger.register(VisionDriveAssistant.getInstance().getLogger());
			robotLogger.register(CargoIntake.getInstance().getLogger());
			robotLogger.register(HatchDeploy.getInstance().getLogger());
			robotLogger.register(Climber.getInstance().getLogger());
    		
    		setInitialPose(new Pose());

   		
    	}
    	catch(Throwable t)
    	{
    		CrashTracker.logThrowableCrash(t);
    		throw t;
    	}
	}
	
	public void setInitialPose (Pose _initialPose){
		robotState.reset(Timer.getFPGATimestamp(), DriveState.getInstance().getLeftDistanceInches(), DriveState.getInstance().getRightDistanceInches(), _initialPose);
    	System.out.println("InitialPose: " + _initialPose);
    }
    
    public void zeroAllSensors()
    {
    	drive.zeroSensors();
    	superStructure.zeroSensors();
		// mSuperstructure.zeroSensors();
    }
    
    public void stopAll()
    {
    	drive.stop();
    	superStructure.stop();
		// mSuperstructure.stop();
    }



	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	@Override
	public void disabledInit()
	{
		operationalMode = OperationalMode.DISABLED;
		boolean logToFile = false;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);
		zeroAllSensors();

		try
		{
			CrashTracker.logDisabledInit();
			if (autoModeExecuter != null)
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			stopAll(); // stop all actuators
			loopController.start();
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic()
	{
		try
		{
			stopAll(); // stop all actuators

			cargoCamera.disabledPeriodic();
			hatchCamera.disabledPeriodic();
			   
			// System.gc(); // runs garbage collector
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}



	/****************************************************************
	 * AUTONOMOUS MODE
	 ****************************************************************/

	@Override
	public void autonomousInit() {
    	operationalMode = OperationalMode.AUTONOMOUS;
    	boolean logToFile = false;
    	boolean logToSmartDashboard = true;
    	robotLogger.setOutputMode(logToFile, logToSmartDashboard);

    	try
    	{
			zeroAllSensors();

			superStructure.enable();
			
			CrashTracker.logAutoInit();
			
			selectedJoystick.update();
			cargoCamera.autoInit();
			hatchCamera.autoInit();
	
			
			if (autoModeExecuter != null)
			{
    			autoModeExecuter.stop();
    		}
    		autoModeExecuter = null;
    		
			autoModeExecuter = new AutoModeExecuter();
			// autoModeExecuter.setAutoMode( smartDashboardInteractions.getAutoModeSelection() );
			autoModeExecuter.setAutoMode(new DriveStraightMode());


			setInitialPose( smartDashboardInteractions.getStartPosition() );
 
			autoModeExecuter.start();
    	}
    	catch(Throwable t)
    	{
    		CrashTracker.logThrowableCrash(t);
    		throw t;
    	}
		
	}

	@Override
	public void autonomousPeriodic() 
	{
		// autoTeleopPeriodic();
	}
	
	
	/****************************************************************
	 * TELEOP MODE
	 ****************************************************************/

	@Override
	public void teleopInit(){
		operationalMode = OperationalMode.TELEOP;
		boolean logToFile = false;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard); 

		
		try 
		{
			CrashTracker.logTeleopInit();
			
			// Select joystick control method
			selectedJoystick.update();
			cargoCamera.teleopInit();
			hatchCamera.teleopInit();
			
			// Configure looper
			loopController.start();
			superStructure.enable();
			
			if(autoModeExecuter != null){
    			autoModeExecuter.stop();
    		}
    		
			drive.setOpenLoop(DriveCommand.COAST());
		} 
		catch (Throwable t) 
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void teleopPeriodic() 
	{
		autoTeleopPeriodic();
	}

	public void autoTeleopPeriodic() 
	{
		try
		{
			DriveCommand driveCmd = controls.getDriveCommand();
			drive.setOpenLoop(driveCmd);
			driveCmd = visionDriveAssistant.assist(driveCmd, controls.getButton(Constants.kVisionAssistanceButton));

			//modify drive controls based on buttons
			DriveCommand driveCmdReverse = controlsReverse.run( driveCmd, Constants.kControlsReverseButton);
			drive.setOpenLoop(driveCmdReverse);

			// turn on LEDs in direction of forward travel
			if (CargoIntake.getInstance().shouldBlink()) {  
				 cargoCamera.setLEDMode(Limelight.LedMode.kBlink);
				 hatchCamera.setLEDMode(Limelight.LedMode.kBlink);
			}
			else if (selectedJoystick.getDrivingCargo())
			{
				cargoCamera.setLEDMode(Limelight.LedMode.kOn);
				hatchCamera.setLEDMode(Limelight.LedMode.kOff);
			}
			else
			{
				cargoCamera.setLEDMode(Limelight.LedMode.kOff);
				hatchCamera.setLEDMode(Limelight.LedMode.kOn);
			}
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}


	
	/****************************************************************
	 * TEST MODE
	 ****************************************************************/

	@Override
	public void testInit() 
	{
		loopController.start();
	}

	@Override
	public void testPeriodic()
	{
		drive.testDriveSpeedControl();
	}
	
	
	// called after disabledPeriodic, autoPeriodic, and teleopPeriodic 
	@Override
	public void robotPeriodic()
	{
		loopController.run();		
		robotLogger.log();
	}


	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("OperationalMode", operationalMode.getVal());
        }
    };
    
    public DataLogger getLogger() { return logger; }

	

}


