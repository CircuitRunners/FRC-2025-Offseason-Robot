package frc.robot.auto.autos.doubleSwipe;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class DelayTestAuto extends AutoModeBase {

    public DelayTestAuto(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "Delay Test");

        AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake", mirrorY);
        
        Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

        prepRoutine(
                Commands.runOnce(() -> DataLogManager.log("The auto started")),
                AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
               leftTrenchToNeutralIntake.cmd());
    }
}








