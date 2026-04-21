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

public class LeftBeen extends AutoModeBase {

    public LeftBeen(Drive drive, Superstructure superstructure, AutoFactory factory) {
        super(drive, superstructure, factory, "silly left");

        AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot");
        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake");
        AutoTrajectory leftShootToSilly = trajectory("leftShootToSilly", 0);
        AutoTrajectory leftBumpyIntakeToShoot0 = trajectory("leftBumpyIntakeToShoot", 0);
        AutoTrajectory leftBumpyIntakeToShoot1 = trajectory("leftBumpyIntakeToShoot", 1);

        AutoTrajectory leftBumpyIntakeToShoot2 = trajectory("leftBumpyIntakeToShoot", 2);
        AutoTrajectory leftBumpyIntakeToShoot3 = trajectory("leftBumpyIntakeToShoot", 3);
        AutoTrajectory leftBumpySillyToShoot = trajectory("leftBumpySillyToShoot");





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







