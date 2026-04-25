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
                        { 3.7, 23.0 }, // trench
                        //{ 3.85, 23.0 }, // depot
                        { 4.0, 24.0 },
                        { 4.4, 26.0 },
                        { 4.7, 28.0 },
                        { 5.0, 30.0 }, // might be higher
                        { 5.3, 30.0 } // corner
        };

        public static double[][] kFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        { 1.43, 1625}, // hub
                        { 1.7, 1650}, // corner hub
                        { 2.15, 1775}, // maybe 1725 or 1700
                        { 2.7, 1865}, // maybe 1825
                        { 3.1, 1975}, // tower // might be higher
                        { 3.7, 2125}, // trench
                        { 4.0, 2175},
                        //{ 3.85, 2100}, // depot
                        { 4.4, 2200}, //
                        { 4.7, 2265},
                        { 5.0, 2315}, //
                        { 5.3, 2415} // corner 
        };

        public static double[][] kTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 1.61, 0.95 },
                        { 1.88, 1.14 },
                        { 3.15, 1.16 },
                        { 4.55, 1.17 },
                        { 5.68, 1.21 }
        };
}