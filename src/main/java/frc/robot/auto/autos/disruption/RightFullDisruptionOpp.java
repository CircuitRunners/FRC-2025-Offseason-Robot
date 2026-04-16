package frc.robot.auto.autos.disruption;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RightFullDisruptionOpp extends AutoModeBase {

    public RightFullDisruptionOpp(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "Right Full Disruption Opp");

        AutoTrajectory disruption = trajectory("disruption").mirrorY();
        AutoTrajectory disruptionReturn = trajectory("disruptionReturnOpposite").mirrorY();
        AutoTrajectory rightShootToSilly = trajectory("leftShootToSilly");

        Pose2d startPose = disruption.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        disruption.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithAccuracy(disruptionReturn),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.shootWhenReadyPulse().withTimeout(2.0),
                Commands.deadline(
                        cmdWithAccuracy(rightShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady(),
                superstructure.deployIntake(),
                Commands.deadline(
                        cmdWithAccuracy(rightShootToSilly),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())));
    }
}






