package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class AutoHelpers {
	
    /** Resets the current drivetrain pose to a given one
     * @param pose the pose to reset to
     * @return the command to reset the pose
     */
    public static Command resetPoseIfWithoutEstimate(Pose2d pose, Drive drive) {
		return Commands.runOnce(() -> drive.getDrivetrain().resetPose(pose));
	}

}
