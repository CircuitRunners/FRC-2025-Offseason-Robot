package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

//import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;
import frc.robot.RobotConstants;

public class DriveConstants {
    public static final double kDriveMaxAngularRate = 8.2; // 254
    public static final AngularVelocity kMaxAngularRate = Units.RadiansPerSecond.of(2.75 * Math.PI); // 1678
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
    public static final double kDriveMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 254
    public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts; // 1678
	public static final LinearVelocity kMaxSpeedTippy = kMaxSpeed.div(2.0);
    public static final LinearVelocity kMaxSpeedVeryTippy = kMaxSpeed.div(2.9);
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kSteerJoystickDeadband = 0.05;
    public static final double kHeadingControllerP = 5.0;
    public static final double kHeadingControllerI = 0;
    public static final double kHeadingControllerD = 0;
    public static final double kMidlineBuffer = 1.0;

    public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(12.0);
    public static final LinearAcceleration kMaxAccelTippy = kMaxAcceleration.div(3.0);
    public static final Time elevatorRaiseTimeL4 = Units.Seconds.of(0.65);


	public static final SynchronousPIDF mAutoAlignTippyHeadingController = getAutoAlignTippyHeadingController();
	public static final SynchronousPIDF mAutoAlignTippyTranslationController = getAutoAlignTippyTranslationController();

	public static final SynchronousPIDF mAutoAlignVeryTippyHeadingController = getAutoAlignVeryTippyHeadingController();
	public static final SynchronousPIDF mAutoAlignVeryTippyTranslationController =
			getAutoAlignVeryTippyTranslationController();





    public static final Translation2d kTranslation2dZero = new Translation2d(0.0, 0.0);
    public static final Rotation2d kRotation2dZero = new Rotation2d();


    public static final SynchronousPIDF mAutoAlignHeadingController = getAutoAlignHeadingController();
	public static final SynchronousPIDF mAutoAlignTranslationController = getAutoAlignTranslationController();

	public static final LinearAcceleration kMaxAccelVeryTippy = kMaxAcceleration.div(5.0);
	public static final Time elevatorRaiseTimeNet = Units.Seconds.of(0.75);
	public static final Distance distanceToStartSlowingNet = Units.Meters.of(Util.calculateDistanceToStartDeccel(
			kMaxSpeed.in(Units.MetersPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			kMaxAccelVeryTippy.in(Units.MetersPerSecondPerSecond),
			elevatorRaiseTimeNet.in(Units.Seconds)));
	public static final Distance distanceToRaiseElevatorNet = Units.Meters.of(Util.calculatDistanceToRaiseElevator(
			kMaxAccelVeryTippy.in(Units.MetersPerSecondPerSecond), elevatorRaiseTimeNet.in(Units.Seconds)));

	public static final Distance distanceToStartSlowingL4 = Units.Meters.of(Util.calculateDistanceToStartDeccel(
			kMaxSpeed.in(Units.MetersPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			kMaxAccelTippy.in(Units.MetersPerSecondPerSecond),
			elevatorRaiseTimeL4.in(Units.Seconds)));
	public static final Distance distanceToRaiseElevatorL4 = Units.Meters.of(Util.calculatDistanceToRaiseElevator(
			kMaxAccelTippy.in(Units.MetersPerSecondPerSecond), elevatorRaiseTimeL4.in(Units.Seconds)));

	public static final Distance distanceToStartSlowingL3 = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL3 = Units.Meters.of(999.0);

	public static final Distance distanceToStartSlowingL2 = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL2 = Units.Meters.of(999.0);

	public static final Time elevatorRaiseTimeL1 = Units.Seconds.of(0.4);
	public static final Distance distanceToStartSlowingL1 = Units.Meters.of(Util.calculateDistanceToStartDeccel(
			kMaxSpeed.in(Units.MetersPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond),
			elevatorRaiseTimeL1.in(Units.Seconds)));
	public static final Distance distanceToRaiseElevatorL1 = Units.Meters.of(Util.calculatDistanceToRaiseElevator(
			kMaxAcceleration.in(Units.MetersPerSecondPerSecond), elevatorRaiseTimeL1.in(Units.Seconds)));

	public static final Distance distanceToStartSlowingL2Algae = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL2Algae = Units.Meters.of(999.0);

	public static final Distance distanceToStartSlowingL3Algae = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorL3Algae = Units.Meters.of(999.0);

	public static final Time elevatorRaiseTimeProcessor = Units.Seconds.of(0.5);
	public static final Distance distanceToStartSlowingProcessorAlgae = Units.Meters.of(999.0);
	public static final Distance distanceToRaiseElevatorProcessorAlgae = Units.Meters.of(999.0);


	public static final SynchronousPIDF mStayOnLineTranslationController = getAutoAlignTippyTranslationController();
	public static final SynchronousPIDF mStayOnLineHeadingController = getAutoAlignTippyHeadingController();
    
    // public static final RobotConfig robotConfig;
    // static {
    //     RobotConfig tempConfig = null;
    //     try{
    //         tempConfig = RobotConfig.fromGUISettings();
    //     } catch (Exception e) {
    //         e.printStackTrace();
    //     }
    //     robotConfig = tempConfig;
    // }

    

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

    public static final SwerveRequest.FieldCentric PIDToPoseRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed
					.times(kSteerJoystickDeadband)
					.div(10.0))
			.withRotationalDeadband(DriveConstants.kMaxAngularRate
					.times(kSteerJoystickDeadband)
					.div(10.0))
			.withDriveRequestType(DriveRequestType.Velocity);


	public static SynchronousPIDF getTippyTranslationControllerForLevel(Level level) {
		return switch (level) {
			case L1,
					L2,
					L2_ALGAE,
					L3,
					L3_ALGAE,
					PROCESSOR_ALGAE -> mAutoAlignTranslationController; // not actually tippy so just return normal
			case L4 -> mAutoAlignTippyTranslationController; // elevator is high so use tippy controller
			case NET -> mAutoAlignVeryTippyTranslationController;
			default -> null;
		};
	}

	public static SynchronousPIDF getTippyHeadingControllerForLevel(Level level) {
		return switch (level) {
			case L1,
					L2,
					L2_ALGAE,
					L3,
					L3_ALGAE,
					PROCESSOR_ALGAE -> mAutoAlignHeadingController; // not actually tippy so just return normal
			case L4 -> mAutoAlignTippyHeadingController; // elevator is high so use tippy controller
			case NET -> mAutoAlignVeryTippyHeadingController;
			default -> null;
		};
	}

	private static SynchronousPIDF getAutoAlignHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.0, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.15, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeed.in(Units.MetersPerSecond));
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTippyHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.0, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.times(0.5).in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTippyTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(2.6, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeedTippy.in(Units.MetersPerSecond));
		return controller;
	}

	private static SynchronousPIDF getAutoAlignVeryTippyHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.2, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.times(0.25).in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignVeryTippyTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(2.8, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeedVeryTippy.in(Units.MetersPerSecond));
		return controller;
	}

	public static Distance getDistanceToStartSlowingForLevel(Level level) {
		return switch (level) {
			case L1 -> distanceToStartSlowingL1;
			case L2 -> distanceToStartSlowingL2;
			case L3 -> distanceToStartSlowingL3;
			case L4 -> distanceToStartSlowingL4;
			case L2_ALGAE -> distanceToStartSlowingL2Algae;
			case L3_ALGAE -> distanceToStartSlowingL3Algae;
			case PROCESSOR_ALGAE -> distanceToStartSlowingProcessorAlgae;
			case NET -> distanceToStartSlowingNet;
			default -> null;
		};
	}

	public static Distance getDistanceToRaiseElevatorForLevel(Level level) {
		return switch (level) {
			case L1 -> distanceToRaiseElevatorL1;
			case L2 -> distanceToRaiseElevatorL2;
			case L3 -> distanceToRaiseElevatorL3;
			case L4 -> distanceToRaiseElevatorL4;
			case L2_ALGAE -> distanceToRaiseElevatorL2Algae;
			case L3_ALGAE -> distanceToRaiseElevatorL3Algae;
			case PROCESSOR_ALGAE -> distanceToRaiseElevatorProcessorAlgae;
			case NET -> distanceToRaiseElevatorNet;
			default -> null;
		};
	}

	public static LinearVelocity getVelocityToRaiseForLevel(Level level) {
		return switch (level) {
			case L1, L2, L2_ALGAE, PROCESSOR_ALGAE -> kMaxSpeed.times(1.1); // not actually tippy so just return normal
			case L4, L3, L3_ALGAE -> kMaxSpeedTippy.times(1.1); // elevator is high so use tippy controller
			case NET -> kMaxSpeedVeryTippy.times(1.1);
			default -> null;
		};
	}

    
}
