package frc.robot.shooting;

public class PassingRegression {
        public static double[][] kPassingHoodManualAngle = {

                        // @x --> distance from target (in meters)
                        // @y --> hood angle (in degrees)

                        { 5.46, 38.0 },
                        { 6.62, 38.0 },
                        { 7.8,  38.0 },
                        { 17.16, 38.0}
        };

        public static double[][] kPassingFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        { 5.46, 2050.0 },
                        { 6.62, 2300.0 },
                        { 7.8,  2450.0 },
                        { 17.16,4250.0 }
        };

        public static double[][] kPassingTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 5.46, 1.27 },
                        { 6.62, 1.39 },
                        { 7.8 , 1.49 },
                        { 11.0, 1.75 },
                        { 13.0, 1.76 },
                        { 17.16,2.16 }
        };
    }