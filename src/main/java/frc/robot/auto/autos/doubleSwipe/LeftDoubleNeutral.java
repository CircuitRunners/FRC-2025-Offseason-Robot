package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class LeftDoubleNeutral extends AutoModeBase {

    public LeftDoubleNeutral(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Left Double Neutral");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot");
        AutoTrajectory leftIntakeToShoot2 = trajectory("leftIntakeToShoot2");
        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake");
        AutoTrajectory leftShootToNeutralIntake = trajectory("leftShootToNeutralIntake");

        Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        leftTrenchToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithAccuracy(leftIntakeToShoot),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady(),
                Commands.deadline(
                        leftShootToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithAccuracy(leftIntakeToShoot2),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.shootWhenReadyPulse());
    }
}



