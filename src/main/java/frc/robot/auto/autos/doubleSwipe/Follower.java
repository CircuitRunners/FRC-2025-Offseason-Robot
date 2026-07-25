package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Follower extends AutoModeBase {

    public Follower(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "follower");

        AutoTrajectory intakeToBumpStart = trajectory("follower", 0, mirrorY);
        AutoTrajectory firstBumpStartToEnd = trajectory("follower", 1, mirrorY);
        AutoTrajectory returnToNeutral = trajectory("follower", 2, mirrorY);

        Pose2d startPose = intakeToBumpStart.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.deadline(
                        intakeToBumpStart.cmd().alongWith(Commands.runOnce(() ->superstructure.brakeIntakeRollers(true))),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                firstBumpStartToEnd.cmd(),
                drive.stopDrivetrain(),          
                superstructure.timeoutShootWhenReady(),
                Commands.deadline(
                        cmdWithAccuracy(returnToNeutral),
                        superstructure.runIntakeIfDeployed())
        );  
    }
}







