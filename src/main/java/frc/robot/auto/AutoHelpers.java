package frc.robot.auto;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoHelpers {
    public static Command resetPoseIfWithoutEstimate(Pose2d pose, Drive drive) {
		return Commands.runOnce(() -> drive.getDrivetrain().resetPose(pose));
	}

}
