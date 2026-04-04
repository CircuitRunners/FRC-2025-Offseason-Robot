package frc.robot.shooting;

public class HubRegression {
        public static double[][] kHoodManualAngle = {

                        // @x --> distance from target (in meters)
                        // @y --> hood angle (in degrees)
                        // { 1.43, 11.8 }, // hub
                        // { 1.7, 11.8 }, // corner hub
                        // { 2.15, 13.0},
                        // { 2.7, 13.0 },
                        // { 3.1, 14.5 }, // tower
                        // { 3.5, 15.5 }, // trench
                        // { 3.85, 16.0 }, // depot
                        // { 4.4, 17.0 },
                        // { 5.0, 20.0 },
                        // { 5.3, 22.0 } // corner
                        { 1.43, 20.0 }, // hub
                        { 1.7, 20.0 }, // corner hub
                        { 2.15, 20.0},
                        { 2.7, 20.0 },
                        { 3.1, 20.0 }, // tower
                        { 3.5, 21.2 }, // trench
                        { 3.85, 22.2 }, // depot
                        { 4.4, 23.2 },
                        { 5.0, 27.2 },
                        { 5.3, 28.2 } // corner
        };

        public static double[][] kFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        // { 1.43, 1700}, // hub
                        // { 1.85, 1750}, // corner hub
                        // { 2.15, 1750},
                        // { 2.7, 1850}, //
                        // { 3.1, 1950}, // tower
                        // { 3.5, 2000}, // trench
                        // { 3.85, 2100}, // depot
                        // { 4.4, 2200}, //
                        // { 5.0, 2250}, //
                        // { 5.3, 2300} // corner
                        { 1.43, 1625}, // hub
                        { 1.85, 1625}, // corner hub
                        { 2.15, 1700},
                        { 2.7, 1800},
                        { 3.1, 1900}, // tower
                        { 3.5, 2000}, // trench
                        { 3.85, 2100}, // depot
                        { 4.4, 2200}, //
                        { 5.0, 2300}, //
                        { 5.3, 2350} // corner 
        };

        public static double[][] kTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 1.61, 0.90 },
                        { 1.88, 1.09 },
                        { 3.15, 1.11 },
                        { 4.55, 1.12 },
                        { 5.68, 1.16 }
        };
}