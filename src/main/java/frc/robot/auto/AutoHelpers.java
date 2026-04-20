package frc.robot.auto;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoHelpers {
    public static Command resetPoseIfWithoutEstimate(Pose2d pose, Drive drive) {
		return Commands.runOnce(() -> drive.getDrivetrain().resetPose(pose));
	}



	public static Double launchOnTheMoveOmega(Drive drive) {
	
		final var parameters = ShotCalculator.getInstance(drive).getParameters();
		return MathUtil.clamp(
			parameters.driveVelocity()
				+ (parameters.heading().minus(drive.getRotation()).getRadians()
					* DriveConstants.kHeadingLockControllerP)
				+ ((parameters.driveVelocity()
						- drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond)
					* DriveConstants.kHeadingLockControllerD),
			DriveConstants.kMaxAngularRate.unaryMinus().in(Units.RadiansPerSecond),
			DriveConstants.kMaxAngularRate.in(Units.RadiansPerSecond));
 	}

}
