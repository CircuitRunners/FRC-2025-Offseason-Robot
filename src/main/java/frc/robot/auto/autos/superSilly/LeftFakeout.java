package frc.robot.auto.autos.superSilly;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class LeftFakeout extends AutoModeBase {

    public LeftFakeout(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Left Fakeout");

        AutoTrajectory rightIntakeToShoot = trajectory("leftIntakeToShoot").mirrorY();
        AutoTrajectory rightShootToSilly = trajectory("leftShootToSilly").mirrorY();
        AutoTrajectory leftTrenchFakeout = trajectory("leftTrenchFakeout");

        Pose2d startPose = leftTrenchFakeout.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        leftTrenchFakeout.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithLessAccuracy(rightIntakeToShoot),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        ,
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady(),
                Commands.deadline(
                        cmdWithAccuracy(rightShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady(),
                superstructure.deployIntake(),
                Commands.deadline(
                        cmdWithAccuracy(rightShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())));
    }
}






