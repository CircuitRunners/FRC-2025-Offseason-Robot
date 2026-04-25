package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.drive.SOTMTrajectory;
import frc.robot.Robot;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class GreedyBean extends AutoModeBase {

    public GreedyBean(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "Greedy Bean");

        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake", mirrorY);

        AutoTrajectory firstIntakeToBumpStart = trajectory("leftGreedyBumpyIntakeToSilly", 0, mirrorY);
        AutoTrajectory firstBumpStartToEnd = trajectory("leftGreedyBumpyIntakeToSilly", 1, mirrorY);
        AutoTrajectory firstSOTM = trajectory("leftGreedyBumpyIntakeToSilly", 2, mirrorY);
        AutoTrajectory silly = trajectory("leftGreedyBumpyIntakeToSilly", 3, mirrorY);
        AutoTrajectory secondBumpStartToEnd = trajectory("leftGreedyBumpyIntakeToSilly", 4, mirrorY);
        AutoTrajectory secondSOTM = trajectory("leftGreedyBumpyIntakeToSilly", 5, mirrorY);
        AutoTrajectory goBack = trajectory("leftGreedyBumpyIntakeToSilly", 6, mirrorY);

        Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        leftTrenchToNeutralIntake.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(false)),
                                superstructure.runIntakeIfDeployed())),
                        Commands.deadline(
                                firstIntakeToBumpStart.cmd(),
                                superstructure.runIntakeIfDeployed()),
                    firstBumpStartToEnd.cmd(),           
                Commands.deadline(
                        new SOTMTrajectory(
                                firstSOTM.getRawTrajectory(),
                                drive,
                                superstructure),
                        superstructure.shootWhenReadyRise()),
                Commands.deadline(
                        silly.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                        secondBumpStartToEnd.cmd(),
                Commands.deadline(
                        new SOTMTrajectory(
                                secondSOTM.getRawTrajectory(),
                                drive,
                                superstructure),
                        Commands.waitSeconds(0.3).andThen(superstructure.shootWhenReadyRise())),
                Commands.deadline(
                        goBack.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed()))
        );
                
    }
}







