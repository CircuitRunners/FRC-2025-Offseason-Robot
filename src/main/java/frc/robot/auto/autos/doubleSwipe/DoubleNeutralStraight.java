package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class DoubleNeutralStraight extends AutoModeBase {

    public DoubleNeutralStraight(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "Left Straight");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot", mirrorY);
        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntakeStraight", mirrorY);
        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly", mirrorY);

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
                                cmdWithLessAccuracy(leftIntakeToShoot),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        ,
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReadyRise(),
                Commands.deadline(
                        cmdWithAccuracy(leftShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReadyRise(),
                superstructure.deployIntake(),
                Commands.deadline(
                        cmdWithAccuracy(leftShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())));
    }
}






