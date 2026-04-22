package frc.robot.auto.autos.centerPreload;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.robot.auto.AutoConstants;
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

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(AutoConstants.centerPreloadStart, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        centerToDepot.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                depotToCenter.cmd(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady()
                );
    }
}


