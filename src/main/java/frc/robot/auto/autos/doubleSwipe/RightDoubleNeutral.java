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

public class RightDoubleNeutral extends AutoModeBase {

    public RightDoubleNeutral(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Right Double Neutral");

        AutoTrajectory rightIntakeToShoot = trajectoryMirroredLeftRight("leftIntakeToShoot");
        AutoTrajectory rightIntakeToShoot2 = trajectoryMirroredLeftRight("leftIntakeToShoot2");
        AutoTrajectory rightTrenchToNeutralIntake = trajectoryMirroredLeftRight("leftTrenchToNeutralIntake");
        AutoTrajectory rightShootToNeutralIntake = trajectoryMirroredLeftRight("leftShootToNeutralIntake");

        Pose2d startPose = rightTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        rightTrenchToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithAccuracy(rightIntakeToShoot),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.timeoutShootWhenReady(),
                Commands.deadline(
                        rightShootToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithAccuracy(rightIntakeToShoot2),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.shootWhenReadyTeleop());
    }
}



