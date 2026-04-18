package frc.robot.auto;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.util.Stopwatch;
import frc.robot.auto.AutoConstants.AutoType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

import frc.lib.drive.PIDToPoseCommand;
public class AutoModeBase {
    private static AutoRoutine routine;
	private static Stopwatch stopwatch = new Stopwatch();
	private static AutoType side;
	private static Drive drive;
	private static Superstructure superstructure;


	public AutoModeBase(Drive drive, Superstructure superstructure, AutoFactory factory, String name) {
		routine = factory.newRoutine(name);
		AutoModeBase.drive = drive;
		AutoModeBase.superstructure = superstructure;
	}

	public AutoModeBase(Drive drive, Superstructure superstructure, AutoFactory factory, String name, AutoType side) {
		this(drive, superstructure, factory, name);
		AutoModeBase.side = side;
	}

	/**
	 * @return Trajectory from choreo
	 */
	public AutoTrajectory trajectory(String name) {
		return routine.trajectory(name);
	}

	public AutoTrajectory trajectory(String name, int index) {
		return routine.trajectory(name, index);
	}

	/**
	 * Runs an accuracy-based command for choreo following
	 *
	 * @param trajectory
	 * @param timeout
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory, Time timeout, Distance epsilonDist) {
		return Commands.defer(
						() -> {
							Command trajectoryCmd = trajectory.cmd();
							Timer elapsedTimer = new Timer();
							double totalTime = trajectory.getRawTrajectory().getTotalTime();
							double computedMinFinishTime =
									Math.max(
											AutoConstants.kAccuracyMinGuardTime.in(Units.Seconds),
											totalTime * AutoConstants.kAccuracyMinCompletionFraction);
							final double minFinishTime =
									totalTime > 0.02 ? Math.min(computedMinFinishTime, totalTime - 0.02) : 0.0;

							return new FunctionalCommand(
									() -> {
										elapsedTimer.restart();
										trajectoryCmd.initialize();
									},
									trajectoryCmd::execute,
									(interrupted) -> {
										trajectoryCmd.end(interrupted);
										elapsedTimer.stop();
									},
									() -> elapsedTimer.hasElapsed(minFinishTime) && isFinished(trajectory, epsilonDist));
						},
						Set.of(drive))
				.beforeStarting(() -> superstructure.setDriveReady(false))
				.withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
	}

	/**
	 * Returns an accuracy-based command for choreo following with a settable tolerance
	 *
	 * @param trajectory
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory, Distance epsilonDist) {
		return cmdWithAccuracy(trajectory, AutoConstants.kDefaultTrajectoryTimeout, epsilonDist);
	}

	public static Command cmdWithLessAccuracy(AutoTrajectory trajectory) {
		return cmdWithAccuracy(trajectory, AutoConstants.kAutoLessAccurateLinearEpsilon);
	}

	/**
	 * Returns an accuracy-based command for choreo following, including the default timeout
	 *
	 * @param trajectory
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory) {
		return cmdWithAccuracy(trajectory, AutoConstants.kAutoLinearEpsilon);
	}

	private static boolean rotationIsFinished(AutoTrajectory trajectory) {
		Pose2d currentPose = drive.getPose();
		Pose2d finalPose = trajectory.getFinalPose().get();
		Angle epsilonAngle = AutoConstants.kAutoAngleEpsilon;

		return MathUtil.angleModulus(Math.abs(
						currentPose.getRotation().minus(finalPose.getRotation()).getRadians()))
				< epsilonAngle.in(Units.Radians);
	}

	private static boolean translationIsFinished(AutoTrajectory trajectory, Distance epsilonDist) {
		Pose2d currentPose = drive.getPose();
		Pose2d finalPose = trajectory.getFinalPose().get();

		SmartDashboard.putNumber(
				"Choreo/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters);
	}

	private static boolean isFinished(AutoTrajectory trajectory, Distance epsilonDist) {
		boolean translationCompleted = translationIsFinished(trajectory, epsilonDist);
		boolean rotationCompleted = rotationIsFinished(trajectory);

		SmartDashboard.putBoolean("Choreo/Translation Completed", translationCompleted);
		SmartDashboard.putBoolean("Choreo/Rotation Completed", rotationCompleted);

		if (translationCompleted && rotationCompleted) {
			stopwatch.startIfNotRunning();
			if (stopwatch.getTime().gte(AutoConstants.kDelayTime)) {
				stopwatch.reset();
				return true;
			}
		} else if (!translationCompleted || !rotationCompleted) {
			stopwatch.reset();
		}

		SmartDashboard.putNumber("Choreo/Stopwatch Time", stopwatch.getTimeAsDouble());
		return false;
	}

	public void prepRoutine(Command... sequence) {
		routine.active()
				.onTrue(Commands.sequence(superstructure.getAutoWaitCommand(), Commands.sequence(sequence))
						.withName("Auto Routine Sequential Command Group"));
	}

	public AutoRoutine getRoutine() {
		return routine;
	}
}
