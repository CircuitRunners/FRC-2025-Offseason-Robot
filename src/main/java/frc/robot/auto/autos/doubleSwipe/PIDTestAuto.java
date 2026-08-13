package frc.robot.auto.autos.doubleSwipe;

import frc.lib.drive.PIDToPoseCommand;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class PIDTestAuto extends AutoModeBase {

    public PIDTestAuto(Drive drive, Superstructure superstructure, AutoFactory factory, boolean mirrorY) {
        super(drive, superstructure, factory, "PID Test");

        prepRoutine(
                Commands.runOnce(() -> DataLogManager.log("The auto started")),
                AutoHelpers.resetPoseIfWithoutEstimate(drive.getPose(), drive),
                new PIDToPoseCommand(drive, superstructure, drive.getPose().plus(new Transform2d(3,0,Rotation2d.kZero))));
    }
}








