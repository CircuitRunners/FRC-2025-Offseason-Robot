package frc.robot.auto.autos.superSilly;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class DoubleSilly extends AutoModeBase {

    public DoubleSilly(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "double silly left");

        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToSilly", mirrorY);
        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly", mirrorY);

        Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        cmdWithAccuracy(leftTrenchToNeutralIntake).alongWith(Commands.runOnce(() ->superstructure.brakeIntakeRollers(true))),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                Commands.parallel(
                superstructure.timeoutShootWhenReady(),
                Commands.waitSeconds(1.0).andThen(superstructure.turnToHubAuto().withTimeout(1.0))),
                Commands.deadline(
                        cmdWithAccuracy(leftShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                Commands.parallel(
                superstructure.timeoutShootWhenReady(),
                Commands.waitSeconds(1.0).andThen(superstructure.turnToHubAuto().withTimeout(1.0))),
                superstructure.deployIntake(),
                Commands.deadline(
                        cmdWithAccuracy(leftShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())));
    }
}







