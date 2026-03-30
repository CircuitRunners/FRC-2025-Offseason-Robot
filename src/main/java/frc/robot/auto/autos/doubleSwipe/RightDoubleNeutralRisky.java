package frc.robot.auto.autos.doubleSwipe;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.auto.AutoModeBase;

public class RightDoubleNeutralRisky extends AutoModeBase {

	public RightDoubleNeutralRisky(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "silly right");

        AutoTrajectory rightIntakeToShoot = trajectoryMirroredLeftRight("leftIntakeToShootRisky");

		AutoTrajectory rightTrenchToNeutralIntake = trajectoryMirroredLeftRight("leftTrenchToNeutralIntakeRisky");

        AutoTrajectory leftShootToSilly = trajectoryMirroredLeftRight("leftShootToSilly");

		Pose2d startPose = rightTrenchToNeutralIntake.getInitialPose().get();


		//superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.left);


		prepRoutine(
            AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.deadline(
				rightTrenchToNeutralIntake.cmd(),
				Commands.sequence(
					superstructure.deployIntake(),
					superstructure.runIntakeIfDeployed()
				)
			),
			cmdWithAccuracy(rightIntakeToShoot).alongWith(superstructure.shooterIdleSpinup()),
			drive.stopDrivetrain(),
			superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime),
			Commands.deadline(
				cmdWithAccuracy(leftShootToSilly),
				Commands.sequence(
					superstructure.deployIntake(),
					superstructure.runIntakeIfDeployed(),
                    superstructure.idleIntake()
				)),
                drive.stopDrivetrain(),
			    superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime),
                superstructure.deployIntake()
		);


	}
}