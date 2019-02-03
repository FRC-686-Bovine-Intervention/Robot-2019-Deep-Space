package frc.robot.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
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
	
	public Limelight frontCamera = Limelight.getFrontInstance();

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
		double cameraLatency = frontCamera.getTotalLatencyMs() / 1000.0;
		double imageCaptureTimestamp = currentTime - cameraLatency;		// assumes transport time from phone to this code is instantaneous

		int numTargets = 1;	// for Limelight
		ArrayList<VisionTargetList.Target> targets = new ArrayList<>(numTargets);

		if (frontCamera.getIsTargetFound())
		{
			double hAngle = frontCamera.getTargetHorizontalAngleRad();
			double vAngle = frontCamera.getTargetVerticalAngleRad();
			double hWidth = frontCamera.getHorizontalWidthRad();
			double vWidth = frontCamera.getVerticalWidthRad();
			
			VisionTargetList.Target target = new VisionTargetList.Target(hAngle, vAngle, hWidth, vWidth);
			targets.add(target);
		}

		visionTargetList.set( imageCaptureTimestamp, targets );
	}
	
}
