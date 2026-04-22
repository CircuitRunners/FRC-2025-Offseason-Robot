package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.drive.DriveConstants;

public class AutoConstants {
    public static final double kPThetaController = 5.0;
    public static final double kPXYController = 5.0;
	public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
	public static final Distance kAutoLessAccurateLinearEpsilon = Units.Inches.of(12.0);
	public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
	public static final Time kDelayTime = Units.Milliseconds.of(80);
	public static final Time kDefaultTrajectoryTimeout = Units.Seconds.of(1.0);
	public static final Time kAccuracyMinGuardTime = Units.Seconds.of(0.20);
	public static final double kAccuracyMinCompletionFraction = 0.80;
	public static Pose2d centerPreloadStart = new Pose2d(new Translation2d(3.5490357875823975, 4.020486831665039), Rotation2d.kZero);
	public static Pose2d centerPreloadShoot = new Pose2d(new Translation2d(1.7453689575195312, 4.020486831665039), Rotation2d.kZero);
    public static enum AutoType {
	}

	public static enum AutoEndBehavior {
	}
	public static double shootAllFuelTime = 3.0;
	public static double delayTime = 4.0;

	public static TrajectoryConfig intakeConfig = new TrajectoryConfig(DriveConstants.kIntakeMaxSpeed, DriveConstants.kMaxAcceleration)
		.setEndVelocity(DriveConstants.kIntakeMaxSpeed);
	public static TrajectoryConfig regularConfig = new TrajectoryConfig(DriveConstants.kMaxSpeed, DriveConstants.kMaxAcceleration)
		.setEndVelocity(DriveConstants.kIntakeMaxSpeed);

	public static Pose2d leftShoot = new Pose2d(new Translation2d(3.9356820583343506, 8.069 - 0.55803394317627), Rotation2d.fromRadians(-1.371850369090507));
	public static Pose2d rightShoot = new Pose2d(new Translation2d(3.9356820583343506, 0.55803394317627), Rotation2d.fromRadians(1.371850369090507));



	


}
