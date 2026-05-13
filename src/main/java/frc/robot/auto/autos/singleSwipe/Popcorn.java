package frc.robot.auto.autos.singleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Popcorn extends AutoModeBase {

    public Popcorn(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "popcorn");

        AutoTrajectory bumpStartToEnd = trajectory("leftDisruption2", 0, mirrorY);
        AutoTrajectory disruption2 = trajectory("leftDisruption2", 1, mirrorY);
        AutoTrajectory intakeToBumpStart = trajectory("leftDisruption2ToShootBump", 0, mirrorY);
        AutoTrajectory bumpStartToEnd2 = trajectory("leftDisruption2ToShootBump", 1, mirrorY);

        Pose2d startPose = bumpStartToEnd.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady().withTimeout(1.0),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                bumpStartToEnd.cmd(),
                Commands.deadline(
                        disruption2.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                Commands.sequence(
                        intakeToBumpStart.cmd(),
                        cmdWithLessAccuracy(bumpStartToEnd2))
                        ,
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady());
    }
}







