package frc.robot.auto.autos.centerPreload;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.DriveTrajectory;
import frc.lib.util.FieldLayout;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Asdf extends AutoModeBase {
    public Asdf(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Center Preload");

        AutoTrajectory test = trajectory("NewPath");
        Trajectory<SwerveSample> trajectory = test.getRawTrajectory();
        Supplier<Optional<Double>> omegaSupplier = () -> {
            ShotCalculator.getInstance(drive).clearShootingParameters();
            return Optional.of(AutoHelpers.launchOnTheMoveOmega(drive));
        };

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(FieldLayout.handleAllianceFlip(trajectory.getInitialPose(false).get(), true), drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.runOnce(() -> superstructure.shootOnTheMove = true),
                Commands.deadline(Commands.waitSeconds(0.35), superstructure.shooterIdleSpinup()),
                Commands.deadline(
                        new DriveTrajectory(
                                trajectory,
                                omegaSupplier,
                                drive),
                        superstructure.shootWhenReadyRise()));
    }
}
