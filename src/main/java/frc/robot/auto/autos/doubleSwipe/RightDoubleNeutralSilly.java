package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RightDoubleNeutralSilly extends AutoModeBase {

    public RightDoubleNeutralSilly(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "silly right");

        AutoTrajectory rightIntakeToShoot = trajectory("leftIntakeToShoot").mirrorY();
        AutoTrajectory rightTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake").mirrorY();
        AutoTrajectory rightShootToSilly = trajectory("leftShootToSilly").mirrorY();

        Pose2d startPose = rightTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        rightTrenchToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithLessAccuracy(rightIntakeToShoot),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReadyRise(),
                Commands.deadline(
                        cmdWithLessAccuracy(rightShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReadyRise(),
                superstructure.deployIntake(),
                Commands.deadline(
                        cmdWithLessAccuracy(rightShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())));
    }
}






