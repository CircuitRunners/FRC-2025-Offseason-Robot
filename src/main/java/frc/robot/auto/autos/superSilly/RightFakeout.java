package frc.robot.auto.autos.superSilly;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RightFakeout extends AutoModeBase {

    public RightFakeout(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Left Fakeout");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot");
        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly");
        AutoTrajectory rightTrenchFakeout = trajectory("leftTrenchFakeout").mirrorY();

        Pose2d startPose = rightTrenchFakeout.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        rightTrenchFakeout.cmd(),
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
                        cmdWithAccuracy(leftShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady(),
                superstructure.deployIntake(),
                Commands.deadline(
                        cmdWithAccuracy(leftShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())));
    }
}






