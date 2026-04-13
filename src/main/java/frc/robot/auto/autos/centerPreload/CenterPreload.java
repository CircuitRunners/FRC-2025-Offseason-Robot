package frc.robot.auto.autos.centerPreload;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import choreo.auto.AutoFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class CenterPreload extends AutoModeBase {
    public CenterPreload(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Center Preload");

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(AutoConstants.centerPreloadStart, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        Commands.sequence(
                                new PIDToPoseCommand(drive, superstructure, AutoConstants.centerPreloadShoot),
                                drive.stopDrivetrain(),
                                superstructure.shootWhenReadyTeleop()),
                        superstructure.deployIntake()));
    }
}


