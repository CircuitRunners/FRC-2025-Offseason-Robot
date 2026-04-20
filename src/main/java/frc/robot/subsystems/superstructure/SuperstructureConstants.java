package frc.robot.subsystems.superstructure;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class SuperstructureConstants {
	public static final Time trenchLookaheadTime = Units.Milliseconds.of(100.0);
	public static final Time aimLookaheadTime = Units.Milliseconds.of(100.0);

    public static Distance getAutoAlignScoringDistanceEpsilon() {
		return Units.Inches.of(0);
	}
	
	public static Angle getAutoAlignScoringAngleEpsilon() {
		return Units.Degrees.of(0);
	}

	public static Time getAutoAlignScoringDelay() {
		return Units.Seconds.of(0);
	}
}
