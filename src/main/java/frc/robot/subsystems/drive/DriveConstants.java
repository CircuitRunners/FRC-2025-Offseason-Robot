package frc.robot.subsystems.drive;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

//import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.logging.LogUtil;
import frc.lib.util.SynchronousPIDF;

public class DriveConstants {
    public static final AngularVelocity kMaxAngularRate = Units.RadiansPerSecond.of(8.2 * 0.70);//254 //(2.75 * Math.PI); 1678

    public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
	public static final LinearVelocity kIntakeMaxSpeed = kMaxSpeed.minus(Units.MetersPerSecond.of(2));
    public static final double kSteerJoystickDeadband = 0.05;
	public static final double kDriveJoystickDeadband = 0.05;
	public static final double kHeadingLockControllerP = 5.0;
    public static final double kHeadingLockControllerI = 0.0;
    public static final double kHeadingLockControllerD = 0.0;
    public static final double kMidlineBuffer = 1.0;
	public static final AngularAcceleration kMaxAngularAcceleration =
			kMaxAngularRate.div(0.1).per(Units.Second);

    public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(10.0);

    public static final Translation2d kTranslation2dZero = new Translation2d(0.0, 0.0);
    public static final Rotation2d kRotation2dZero = new Rotation2d();

	public static final Angle driveYawPassToleranceDeg = Units.Degrees.of(10.0);
	public static final Angle driveYawLaunchToleranceDeg = Units.Degrees.of(15.0);
    public static final double kWiggleAmplitudeDeg = 3.0;
    public static final double kWiggleFrequencyHz = 3.0;
    public static final double kWiggleMaxSpeed = 0.5;


    public static final SynchronousPIDF mAutoAlignHeadingController = getAutoAlignHeadingController();
	public static final SynchronousPIDF mAutoAlignTranslationController = getAutoAlignTranslationController();
	public static final SynchronousPIDF mIntakeAutoAlignTranslationController = getIntakeAutoAlignTranslationController();


	public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToPoseRequestUpdater(Drive drive, Pose2d targetPose) {
		return getPIDToPoseRequestUpdater(drive, targetPose, mAutoAlignTranslationController, mAutoAlignHeadingController);
	}

    public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToPoseRequestUpdater(
			Drive drive, Pose2d targetPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		return (SwerveRequest.FieldCentric request) -> {
			Pose2d currentPose = drive.getPose();
			Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.getTranslation());
			LinearVelocity velocity =
					Units.MetersPerSecond.of(translationController.calculate(deltaTranslation.getNorm()));
			Rotation2d velocityDirection = deltaTranslation.getAngle();
			//if (RobotConstants.isRedAlliance) velocityDirection = velocityDirection.plus(Rotation2d.k180deg);

			headingController.setSetpoint(Units.Radians.of(
							MathUtil.angleModulus(targetPose.getRotation().getRadians()))
					.in(Units.Rotations));

			LogUtil.recordPose2d("PID To Pose Updater/Target Pose", targetPose);

			request.withVelocityX(velocity.times(velocityDirection.getCos()))
					.withVelocityY(velocity.times(velocityDirection.getSin()))
					.withRotationalRate(Units.RotationsPerSecond.of(
							headingController.calculate(Units.Radians.of(MathUtil.angleModulus(
											currentPose.getRotation().getRadians()))
									.in(Units.Rotations))));

			return request;
		};
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getDriveToPoseRequestUpdater(Drive drive, Pose2d targetPose) {
		return getDriveToPoseRequestUpdater(drive, targetPose, mAutoAlignTranslationController, mAutoAlignHeadingController);
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getDriveToPoseRequestUpdater(
			Drive drive, Pose2d targetPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		return (SwerveRequest.FieldCentric request) -> {

			Pose2d currentPose = drive.getPose();

			Translation2d error =
				targetPose.getTranslation().minus(currentPose.getTranslation());

			double distance = error.getNorm();


			Translation2d direction =
				distance > 1e-4 ? error.div(distance) : new Translation2d();


			double driveSpeed = kMaxSpeed.in(Units.MetersPerSecond);

			LinearVelocity vx =
				Units.MetersPerSecond.of(direction.getX() * driveSpeed);
			LinearVelocity vy =
				Units.MetersPerSecond.of(direction.getY() * driveSpeed);


			headingController.setSetpoint(
				Units.Radians.of(
					MathUtil.angleModulus(targetPose.getRotation().getRadians())
				).in(Units.Rotations)
			);

			var omega = Units.RotationsPerSecond.of(
				headingController.calculate(
					Units.Radians.of(
						MathUtil.angleModulus(
							currentPose.getRotation().getRadians()
						)
					).in(Units.Rotations)
				)
			);

			request.withVelocityX(vx)
				.withVelocityY(vy)
				.withRotationalRate(omega);

			return request;
		};
	}

    public static final SwerveRequest.FieldCentric PIDToPoseRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed
					.times(kSteerJoystickDeadband)
					.div(10.0))
			.withRotationalDeadband(DriveConstants.kMaxAngularRate
					.times(kSteerJoystickDeadband)
					.div(10.0))
			.withDriveRequestType(DriveRequestType.Velocity);


	



	public static SynchronousPIDF getAutoAlignHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.0, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	public static SynchronousPIDF getAutoAlignTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.50, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeed.in(Units.MetersPerSecond));
		return controller;
	}

	public static SynchronousPIDF getIntakeAutoAlignTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.15, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeed.times(0.2).in(Units.MetersPerSecond));
		return controller;
	}

	public static RobotConfig getRobotConfig() {
		try {
			return RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}
}

	
