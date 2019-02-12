package frc.robot.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.lib.sensors.Limelight;
import frc.robot.loops.Loop;


/**
 * VisionLoop contains the various attributes calculated by the vision system,
 * namely a list of targets and the timestamp at which it was captured.
 */
public class VisionLoop implements Loop
{
    private static VisionLoop instance = new VisionLoop();
    public static VisionLoop getInstance() { return instance; }
	
	// camera selection
	public Limelight cargoCamera = Limelight.getCargoInstance();
	public Limelight hatchCamera = Limelight.getHatchInstance();
	public Robot robot = Robot.getInstance();

	public VisionTargetList visionTargetList = VisionTargetList.getInstance();

	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
    	double currentTime = Timer.getFPGATimestamp();

		// get target info from Limelight
		getTargets(currentTime);
	}

	@Override public void onStop()
	{
		// nothing
	}

	public void getTargets(double currentTime)
	{
		double cameraLatency = cargoCamera.getTotalLatencyMs() / 1000.0;
		double imageCaptureTimestamp = currentTime - cameraLatency;		// assumes transport time from phone to this code is instantaneous

		int numTargets = 1;	// for Limelight
		ArrayList<VisionTargetList.Target> targets = new ArrayList<>(numTargets);

		Limelight cameraSelection = robot.getJoystick().getDrivingForward() ? cargoCamera : hatchCamera;

		if (cameraSelection.getIsTargetFound())
		{
			double hAngle = cameraSelection.getTargetHorizontalAngleRad();
			double vAngle = cameraSelection.getTargetVerticalAngleRad();
			double hWidth = cameraSelection.getHorizontalWidthRad();
			double vWidth = cameraSelection.getVerticalWidthRad();
			
			VisionTargetList.Target target = new VisionTargetList.Target(hAngle, vAngle, hWidth, vWidth);
			targets.add(target);
		}

		visionTargetList.set( imageCaptureTimestamp, targets );
	}
	
}
