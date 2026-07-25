package frc.robot.auto.autos.centerPreload;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class CenterPreloadDepot extends AutoModeBase {
    public CenterPreloadDepot(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Center Preload Depot");

        AutoTrajectory centerToDepot = trajectory("centerToDepot");
        AutoTrajectory depotToCenter = trajectory("depotToCenter");

        Pose2d startPose = centerToDepot.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        centerToDepot.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                Commands.waitSeconds(0.5).alongWith(superstructure.runIntakeIfDeployed().withTimeout(0.5)),
                Commands.parallel(
                                cmdWithLessAccuracy(depotToCenter),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0)),
                drive.stopDrivetrain(),
                Commands.waitSeconds(1),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.shootWhenReadyPulse()
                );
    }
}


