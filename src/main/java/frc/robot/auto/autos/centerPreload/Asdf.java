package frc.robot.auto.autos.centerPreload;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.SOTMTrajectory;
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
        Pose2d startPose = test.getInitialPose().get();
        Trajectory<SwerveSample> trajectory = test.getRawTrajectory();


        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        new SOTMTrajectory(
                                trajectory,
                                drive,
                                superstructure),
                        superstructure.shootWhenReadyRise()));
    }
}
