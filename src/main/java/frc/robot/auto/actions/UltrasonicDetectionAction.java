package frc.robot.auto.actions;


import frc.robot.Constants;
import frc.robot.lib.sensors.UltrasonicSensor;
import frc.robot.lib.util.DataLogger;


public class UltrasonicDetectionAction implements Action 
{
    UltrasonicSensor sensor;
    double distance = 999.0;
    double thresholdDistance = 14.0;
    boolean collisionDetected = false;

    public UltrasonicDetectionAction() 
    {
        sensor = new UltrasonicSensor(Constants.kUltrasonicSensorPort);
    }

    @Override
    public void start() 
    {
        collisionDetected = false;
    }


    @Override
    public void update() 
    {
        // nothing to do.  just waiting for isFinished()
    }	
	
	
    @Override
    public boolean isFinished() 
    {
        distance = sensor.update();
        System.out.printf("US distance = %.1f\n", distance);
    	return (distance <= thresholdDistance);
    }

    @Override
    public void done() 
    {
        // cleanup code, if any
    }

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("HatchCollision/distance", distance );
            put("HatchCollision/enableAngleThresh", thresholdDistance );
        }
    };
     
    public DataLogger getLogger() { return logger; }
}
