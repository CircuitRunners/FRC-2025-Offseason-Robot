package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.drive.SOTMTrajectory;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Bean extends AutoModeBase {

    public Bean(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "silly left");

        AutoTrajectory leftIntakeToShoot = flipY(trajectory("leftIntakeToShoot"), mirrorY);
        AutoTrajectory leftTrenchToNeutralIntake = flipY(trajectory("leftTrenchToNeutralIntake"), mirrorY);
        AutoTrajectory leftShootToSilly = flipY(trajectory("leftShootToSilly", 0), mirrorY);
        AutoTrajectory leftBumpyIntakeToShoot0 = flipY(trajectory("leftBumpyIntakeToShoot", 0), mirrorY);
        AutoTrajectory leftBumpyIntakeToShoot1 = flipY(trajectory("leftBumpyIntakeToShoot", 1), mirrorY);

        AutoTrajectory leftBumpyIntakeToShoot2 = flipY(trajectory("leftBumpyIntakeToShoot", 2), mirrorY);
        AutoTrajectory leftBumpyIntakeToShoot3 = flipY(trajectory("leftBumpyIntakeToShoot", 3), mirrorY);
        AutoTrajectory leftBumpySillyToShoot = flipY(trajectory("leftBumpySillyToShoot"), mirrorY);





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
                Commands.sequence(
                    leftBumpyIntakeToShoot0.cmd(),
                    superstructure.disableCamera(true),
                    leftBumpyIntakeToShoot1.cmd(),
                    superstructure.disableCamera(false)                  
                ),
                Commands.deadline(
                        new SOTMTrajectory(
                                leftBumpyIntakeToShoot2.getRawTrajectory(),
                                drive,
                                superstructure),
                        superstructure.shootWhenReadyRise()),
                leftBumpyIntakeToShoot3.cmd(),
                Commands.deadline(
                        leftShootToSilly.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed())),
                
                Commands.sequence(
                    leftBumpySillyToShoot.cmd(),
                    leftBumpyIntakeToShoot0.cmd(),
                    superstructure.disableCamera(true),
                    leftBumpyIntakeToShoot1.cmd(),
                    superstructure.disableCamera(false)
                ),
                Commands.deadline(
                        new SOTMTrajectory(
                                leftBumpyIntakeToShoot2.getRawTrajectory(),
                                drive,
                                superstructure),
                        superstructure.shootWhenReadyRise()),
                leftBumpyIntakeToShoot3.cmd(),
                Commands.deadline(
                        leftShootToSilly.cmd(),
                        Commands.sequence(
                                superstructure.deployIntake(),
                                superstructure.runIntakeIfDeployed()))
        );
                
    }
}







