package frc.robot.auto.autos.singleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Popcorn extends AutoModeBase {

    public Popcorn(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "silly left");

        AutoTrajectory disruption2 = trajectory("disruption2");
        AutoTrajectory disruption2ToShoot = trajectory("disruption2ToShoot");

        Pose2d startPose = disruption2.getInitialPose().get();

        prepRoutine(
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady().withTimeout(2.25),
                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                Commands.deadline(
                        disruption2.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                Commands.runOnce(() -> superstructure.brakeIntakeRollers(true)),
                                superstructure.runIntakeIfDeployed())),
                Commands.parallel(
                                cmdWithAccuracy(disruption2ToShoot),
                                superstructure.runIntakeIfDeployed().withTimeout(1.0))
                        .alongWith(superstructure.shooterIdleSpinup()),
                drive.stopDrivetrain(),
                superstructure.turnToHubAuto().withTimeout(1.0),
                superstructure.timeoutShootWhenReady());
    }
}







