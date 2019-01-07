package frc.robot.command_status;

import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;
import frc.robot.lib.util.Kinematics;
import frc.robot.lib.util.Pose;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout
 * the match. A coordinate frame is simply a point and direction in space that
 * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
 * spatial relationship between different frames.
 * 
 * Robot frames of interest (from parent to child):
 * 
 * 1. Field frame: origin is where the robot is turned on
 * 
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 * 
 * 3. Turret fixed frame: origin is the center of the turret when the turret is
 * at 0 degrees rotation relative to the vehicle frame
 * 
 * 4. Turret rotating frame: origin is the center of the turret as it rotates
 * 
 * 5. Camera frame: origin is the center of the camera imager as it rotates with
 * the turret
 * 
 * 6. Goal frame: origin is the center of the goal (note that orientation in
 * this frame is arbitrary). Also note that there can be multiple goal frames.
 * 
 * As a kinematic chain with 6 frames, there are 5 transforms of interest:
 * 
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably drift, but is usually accurate over
 * short time periods.
 * 
 * 2. Vehicle-to-turret-fixed: This is a constant.
 * 
 * 3. Vehicle-to-turret-rotating: This is a pure rotation, and is tracked over
 * time using the turret encoder.
 * 
 * 4. Turret-rotating-to-camera: This is a constant.
 * 
 * 5. Camera-to-goal: This is a pure translation, and is measured by the vision
 * system.
 */

public class RobotState 
{
	// singleton class
	private static RobotState instance = null;
	public static RobotState getInstance() 
	{ 
		if (instance == null) {
			instance = new RobotState();
		}
		return instance;
	}

    public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;

    private InterpolatingTreeMap<InterpolatingDouble, Pose> fieldToRobot;
    private Kinematics.LinearAngularSpeed robotSpeed;

    private double gyroCorrection;
    
    private double lPrevDistance = 0;
    private double rPrevDistance = 0;
    
    public RobotState() { reset( 0, 0, 0, new Pose() ); }

	public synchronized void reset(double _startTime, double _lEncoderDistance, double _rEncoderDistance, Pose _initialFieldToRobot) 
	{
		// calibrate initial position to initial pose (set by autonomous mode)
        fieldToRobot = new InterpolatingTreeMap<>(kObservationBufferSize);
        fieldToRobot.put(new InterpolatingDouble(_startTime), _initialFieldToRobot);
       
        // calculate gyro heading correction for the desired initial pose (as set by autonomous mode)
        double desiredHeading = _initialFieldToRobot.getHeading();  
        double gyroHeading  = DriveState.getInstance().getHeading();
        gyroCorrection = gyroHeading - desiredHeading;		// subtract gyroCorrection from actual gyro heading to get desired orientation
        
        robotSpeed = new Kinematics.LinearAngularSpeed(0, 0);
        
        setPrevEncoderDistance(_lEncoderDistance, _rEncoderDistance);
    }
	
	public void setPrevEncoderDistance(double _lPrevDistance, double _rPrevDistance)
	{
        lPrevDistance = _lPrevDistance;
        rPrevDistance = _rPrevDistance;     
	}
	 	
	public synchronized Pose getFieldToVehicle(double _timestamp) 
	{
        return fieldToRobot.getInterpolated(new InterpolatingDouble(_timestamp));
    }

    public synchronized Pose getLatestFieldToVehicle() 
    {
        return fieldToRobot.lastEntry().getValue();
    }

    public synchronized Pose getPredictedFieldToVehicle(double _lookaheadTime) 
    {
    	Kinematics.LinearAngularSpeed speed = new Kinematics.LinearAngularSpeed(robotSpeed.linearSpeed * _lookaheadTime, robotSpeed.angularSpeed * _lookaheadTime);
        return Kinematics.travelArc(getLatestFieldToVehicle(), speed);
    }

    public synchronized void addFieldToVehicleObservation(double _timestamp, Pose _observation)
    {
        fieldToRobot.put(new InterpolatingDouble(_timestamp), _observation);
    }


    public void generateOdometryFromSensors(double _time, double _lEncoderDistance, double _rEncoderDistance, 
    		                                double _lEncoderSpeed, double _rEncoderSpeed, double _gyroAngle) 
    {
        Pose lastPose = getLatestFieldToVehicle();

        // get change in encoder distance from last call
        double lDeltaDistance = _lEncoderDistance - lPrevDistance; 
        double rDeltaDistance = _rEncoderDistance - rPrevDistance;
        
        setPrevEncoderDistance(_lEncoderDistance, _rEncoderDistance);

        Pose odometry = Kinematics.integrateForwardKinematics(lastPose, lDeltaDistance, rDeltaDistance, _gyroAngle - gyroCorrection);
        Kinematics.LinearAngularSpeed speed = Kinematics.forwardKinematics(_lEncoderSpeed, _rEncoderSpeed);
        
        addFieldToVehicleObservation(_time, odometry);	// store odometry
        robotSpeed = speed;								// used in getPredictedFieldToVehicle()
    }

    public double getSpeed()
    {
    	return robotSpeed.linearSpeed;
    }
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        	synchronized (RobotState.this)
        	{
	            Pose odometry = getLatestFieldToVehicle();
	            put("RobotState/positionX",  odometry.getX());
	            put("RobotState/positionY",  odometry.getY());
	            put("RobotState/headingDeg", odometry.getHeadingDeg());
        	}
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
}
