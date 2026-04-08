// package frc.robot.auto.autos.singleSwipe;

// import java.util.Set;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.lib.drive.PIDToPoseCommand;
// import frc.lib.util.FieldLayout;
// import frc.robot.RobotConstants;
// import frc.robot.auto.AutoConstants;
// import frc.robot.auto.AutoHelpers;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.DriveConstants;
// import frc.robot.subsystems.superstructure.Superstructure;
// import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
// import frc.robot.auto.AutoModeBase;

// public class RightObjectDetect extends AutoModeBase {

// 	public RightObjectDetect(Drive drive, Superstructure superstructure, AutoFactory factory) {
// 		super(drive, superstructure, factory, "lghafkjd");

// 		AutoTrajectory rightShootToObjectDetect = trajectoryMirroredLeftRight("leftShootToObjectDetect");

// 		AutoTrajectory rightIntakeToShoot = trajectoryMirroredLeftRight("leftIntakeToShoot");


// 		Pose2d startPose = rightShootToObjectDetect.getInitialPose().get();

// 		superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.RIGHT);


// 		prepRoutine(
// 			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
//             Commands.deadline(
//                 rightShootToObjectDetect.cmd(),
//                 superstructure.deployIntake()
//             ),
// 			Commands.deadline(
// 				superstructure.collectFuelTrajectory(),

// 				superstructure.runIntakeIfDeployed()
// 			),
// 			cmdWithAccuracy(rightIntakeToShoot).alongWith(superstructure.shooterIdleSpinup()),
// 			drive.stopDrivetrain(),
// 			superstructure.shootWhenReadyTeleop()
			
// 		);


// 	}
// }

