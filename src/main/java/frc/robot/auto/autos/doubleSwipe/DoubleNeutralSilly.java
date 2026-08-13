package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class DoubleNeutralSilly extends AutoModeBase {

    public DoubleNeutralSilly(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "[LEFT] Silly");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot", mirrorY);
        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake", mirrorY);
        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly", mirrorY);

        Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                Commands.runOnce(() -> DataLogManager.log("The auto started")),
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.deadline(
                        leftTrenchToNeutralIntake.cmd().alongWith(Commands.runOnce(() ->superstructure.brakeIntakeRollers(true))
                        ),
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







