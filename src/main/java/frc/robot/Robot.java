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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.GoalStates;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.CheckForNewDS;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControlsReversible;
import frc.robot.lib.sensors.Limelight;
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
	SelectedDriverControlsReversible selectedDriverControls = SelectedDriverControlsReversible.getInstance();

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

	Hatch hatch;
	
	OperationalMode operationalMode = OperationalMode.getInstance();


	final boolean PRACTICE_BOT = false;		// set to true when running on practice bot without Cargo Intake / Climber


    public Robot() {
    	CrashTracker.logRobotConstruction();
    }
    
    
	@Override
	public void robotInit() {
		try
    	{
    		CrashTracker.logRobotInit();
			operationalMode.set(OperationalMode.OperationalModeEnum.DISABLED);

			LiveWindow.disableTelemetry(pdp);	// stops CAN error

			hatch = Hatch.getInstance();
    		
    		loopController = new LoopController();
    		loopController.register(drive.getVelocityPIDLoop());
    		loopController.register(DriveLoop.getInstance());
       		loopController.register(RobotStateLoop.getInstance());
    		loopController.register(VisionLoop.getInstance());
			loopController.register(GoalStateLoop.getInstance());
			loopController.register(Hatch.getInstance());
			if (!PRACTICE_BOT)
			{
				loopController.register(CargoIntake.getInstance());
				loopController.register(Climber.getInstance());
			}

            SelectedDriverControlsReversible.getInstance().setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
    		
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
			robotLogger.register(SelectedDriverControlsReversible.getInstance().getLogger());
			// robotLogger.register(Hatch.getInstance().getLogger());
		
			if (!PRACTICE_BOT)
			{
				robotLogger.register(CargoIntake.getInstance().getLogger());
				robotLogger.register(Climber.getInstance().getLogger());
			}
    		
    		setInitialPose(new Pose());

   		
    	}
    	catch(Throwable t)
    	{
    		CrashTracker.logThrowableCrash(t);
    		throw t;
    	}
	}
	
	public void setInitialPose (Pose _initialPose){
		robotState.reset(_initialPose);
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
		operationalMode.set(OperationalMode.OperationalModeEnum.DISABLED);
		boolean logToFile = false;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);
		zeroAllSensors();

		Shuffleboard.stopRecording();

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
		operationalMode.set(OperationalMode.OperationalModeEnum.AUTONOMOUS);
    	boolean logToFile = false;
    	boolean logToSmartDashboard = true;
    	robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		Shuffleboard.startRecording();

    	try
    	{
			zeroAllSensors();

			superStructure.enable();
			
			CrashTracker.logAutoInit();
			
            SelectedDriverControlsReversible.getInstance().setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
			cargoCamera.autoInit();
			hatchCamera.autoInit();
	
            // only using hatch camera during auto
            cargoCamera.setLEDMode(Limelight.LedMode.kOff);
            hatchCamera.setLEDMode(Limelight.LedMode.kOn);

            Shuffleboard.selectTab(hatchCamera.getTableName());     // select hatch camera tab in Shuffleboard
            
			
			if (autoModeExecuter != null)
			{
    			autoModeExecuter.stop();
    		}
    		autoModeExecuter = null;
    		
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode( smartDashboardInteractions.getAutoModeSelection() );
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
        if (operationalMode.get() == OperationalMode.OperationalModeEnum.AUTONOMOUS)
        {
            if (selectedDriverControls.joystickActive())
            {
                // Driver just started using joystick.  Abandon autonomous
                teleopInit();
                teleopPeriodic();
            }

            // if joystick not being used, stay in autonomous
            // nothing else to do in this thread
        }
        else
        {
            // driver has taken us out of autonomous control
            teleopPeriodic();
        }
	}
	
	
	/****************************************************************
	 * TELEOP MODE
	 ****************************************************************/

	@Override
	public void teleopInit(){
		operationalMode.set(OperationalMode.OperationalModeEnum.TELEOP);
		boolean logToFile = false;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard); 

		Shuffleboard.startRecording();
		
		try 
		{
			CrashTracker.logTeleopInit();
			
            // Select joystick control method
            SelectedDriverControlsReversible.getInstance().setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
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
	
	boolean ledsOnlyWhenActive = true;

	@Override
	public void teleopPeriodic() 
	{
		try
		{
            SelectedDriverControlsReversible driverControls = SelectedDriverControlsReversible.getInstance();

			boolean visionButton = driverControls.getBoolean(DriverControlsEnum.VISION_ASSIST);
			boolean ledsActive = !ledsOnlyWhenActive || (ledsOnlyWhenActive && visionButton);

			DriveCommand driveCmd = selectedDriverControls.getDriveCommand();
			driveCmd = visionDriveAssistant.assist(driveCmd, visionButton);

			//modify drive controls based on buttons
			drive.setOpenLoop(driveCmd);

			// turn on LEDs in direction of forward travel
			if (!PRACTICE_BOT && (CargoIntake.getInstance().shouldBlink() || Climber.getInstance().shouldBlink())) 
			{  
				 cargoCamera.setLEDMode(Limelight.LedMode.kBlink);
				 hatchCamera.setLEDMode(Limelight.LedMode.kBlink);
			}
			else if (driverControls.getDrivingCargo() && ledsActive)
			{
				cargoCamera.setLEDMode(Limelight.LedMode.kOn);
				hatchCamera.setLEDMode(Limelight.LedMode.kOff);
			}
			else if (ledsActive)
			{
				cargoCamera.setLEDMode(Limelight.LedMode.kOff);
				hatchCamera.setLEDMode(Limelight.LedMode.kOn);
			}
			else
			{
				cargoCamera.setLEDMode(Limelight.LedMode.kOff);
				hatchCamera.setLEDMode(Limelight.LedMode.kOff);
			}

			// change shuffleboard tabs
			if(CargoIntake.getInstance().state != CargoIntake.CargoDeployStateEnum.CLIMBING)
			{
				if (driverControls.getDrivingCargo())
				{
					Shuffleboard.selectTab(cargoCamera.getTableName());     // select cargo camera tab in Shuffleboard
				}
				else
				{
					Shuffleboard.selectTab(hatchCamera.getTableName());     // select hatch camera tab in Shuffleboard
				}
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
		operationalMode.set(OperationalMode.OperationalModeEnum.TEST);
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
        frc.robot.lib.joystick.CheckForNewDS.getInstance().reset();
	}


	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("OperationalMode", operationalMode.get().toString());
        }
    };
    
    public DataLogger getLogger() { return logger; }

	

}


