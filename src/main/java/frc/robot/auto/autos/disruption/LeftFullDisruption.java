package frc.robot.auto.autos.disruption;

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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.auto.AutoModeBase;

public class LeftFullDisruption extends AutoModeBase {

	public LeftFullDisruption(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Right Double Neutral");

		AutoTrajectory disruption = trajectory("disruption");
		AutoTrajectory disruptionReturn = trajectory("disruptionReturn");

        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly");


		Pose2d startPose = disruption.getInitialPose().get();

		//superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.RIGHT);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			disruption.cmd(),
			Commands.deadline(
				cmdWithAccuracy(disruptionReturn),
				Commands.sequence(
					superstructure.runIntakeIfDeployed()
				)
			),
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