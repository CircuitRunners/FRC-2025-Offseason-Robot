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

public class LeftDoubleNeutralSilly extends AutoModeBase {

	public LeftDoubleNeutralSilly(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "silly left");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot");

		AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake");

        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly");

		Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();


		//superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.left);


		prepRoutine(
            AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.runOnce(() ->superstructure.brakeIntakeRollers(true)),
			Commands.deadline(
				leftTrenchToNeutralIntake.cmd(),
				Commands.sequence(
					superstructure.deployIntake(),
					Commands.runOnce(() ->superstructure.brakeIntakeRollers(true)),
					superstructure.runIntakeIfDeployed()
				)
			),
			Commands.parallel(
			cmdWithAccuracy(leftIntakeToShoot),
			superstructure.runIntakeIfDeployed().withTimeout(1)).alongWith(superstructure.shooterIdleSpinup()),
			drive.stopDrivetrain(),
			superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime),
			Commands.deadline(
				cmdWithAccuracy(leftShootToSilly),
				Commands.sequence(
					superstructure.deployIntake(),
					superstructure.runIntakeIfDeployed()
				)),
                drive.stopDrivetrain(),
			    superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime),
                superstructure.deployIntake(),
				Commands.deadline(
				cmdWithAccuracy(leftShootToSilly),
				Commands.sequence(
					superstructure.deployIntake(),
					superstructure.runIntakeIfDeployed()
				))
		);


	}
}