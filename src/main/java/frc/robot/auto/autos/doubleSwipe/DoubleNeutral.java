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

public class DoubleNeutral extends AutoModeBase {

    public DoubleNeutral(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "Left Double Neutral");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot", mirrorY);
        AutoTrajectory leftIntakeToShoot2 = trajectory("leftIntakeToShoot2", mirrorY);
        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake", mirrorY);
        AutoTrajectory leftShootToNeutralIntake = trajectory("leftShootToNeutralIntake", mirrorY);

        Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.deadline(
                        leftTrenchToNeutralIntake.cmd().alongWith(Commands.runOnce(() ->superstructure.brakeIntakeRollers(true))),
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
                superstructure.timeoutShootWhenReady(),
                Commands.deadline(
                        leftShootToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithLessAccuracy(leftIntakeToShoot2),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        ,
                drive.stopDrivetrain(),
                superstructure.shootWhenReadyPulse());
    }
}



