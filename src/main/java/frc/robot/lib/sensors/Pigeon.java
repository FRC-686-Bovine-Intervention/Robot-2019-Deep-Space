package frc.robot.lib.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;

public class Pigeon extends GyroBase 
{
	// singleton class
        private static GyroBase instance = null;
        public static GyroBase getInstance() 
        { 
            if (instance == null) {
                instance = new Pigeon();
            }
            return instance;
        }

        PigeonIMU pigeon;

        // yaw/picth/roll constants
        final int kYAW = 0;
        final int kPITCH = 1;
        final int kROLL = 2;
        final int kYPR_SIZE = kROLL + 1;

        // constructors
        public Pigeon() 
        {
                pigeon = new PigeonIMU(0);
        }

        /**
         * Returns heading for the GyroBase class.
         *
         */
        public double getHeadingDeg() 
        {
                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                double yaw = ypr[kYAW];
                return yaw;
        }

        public double getPitchDeg()
         {
                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                double pitch = ypr[kPITCH];
                return pitch;
        }

        public double getRollDeg()
         {
                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                double roll = ypr[kROLL];
                return roll;
        }

        @Override
        public void zeroSensor() 
        {
                pigeon.setYaw(0.0, Constants.kTalonTimeoutMs);
        }

}
