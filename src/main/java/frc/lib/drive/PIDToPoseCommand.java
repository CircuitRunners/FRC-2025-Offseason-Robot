package frc.lib.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LogUtil;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class PIDToPoseCommand extends Command {
    Drive drive;
    Superstructure superstructure;
    Pose2d finalPose;
    Rotation2d targetRotation;
    Distance epsilonDist;
    Angle epsilonAngle;
    Util.Pose2dTimeInterpolable interpolable;
    DelayedBoolean atTarget;
    boolean isAuto;
    SynchronousPIDF translationController;
    SynchronousPIDF headingController;

    public PIDToPoseCommand(
            Drive drive,
            Superstructure superstructure,
            Pose2d finalPose,
            Distance epsilonDist,
            Angle epsilonAngle,
            Time delayTime,
            Rotation2d targetRotation,
            SynchronousPIDF translationController,
            SynchronousPIDF headingController) {
        addRequirements(drive);

        this.drive = drive;
        this.superstructure = superstructure;
        this.finalPose = finalPose;
		this.epsilonDist = epsilonDist;
		this.epsilonAngle = epsilonAngle;
		this.targetRotation = targetRotation.plus(Rotation2d.k180deg);
		this.translationController = translationController;
		this.headingController = headingController;

        atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));
    }

    public PIDToPoseCommand(
            Drive drive,
            Superstructure superstructure,
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
        this(
                drive,
                superstructure,
                finalPose,
                epsilonDist,
                epsilonAngle,
                delayTime,
                finalPose.getRotation(),
                translationController,
                headingController);
    }

    public PIDToPoseCommand(
            Drive drive,
            Superstructure superstructure,
            Pose2d finalPose,
            Time delayTime,
            Rotation2d targetRotation,
            SynchronousPIDF translationController,
            SynchronousPIDF headingController) {
        this(
                drive,
                superstructure,
                finalPose,
                Units.Centimeters.of(4),
                Units.Degrees.of(.8),
                delayTime,
                targetRotation,
                translationController,
                headingController);
    }

    public PIDToPoseCommand(
            Drive drive,
            Superstructure superstructure,
			Pose2d finalPose,
			Time delayTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(drive, superstructure, finalPose, delayTime, finalPose.getRotation(), translationController, headingController);
	}

    /**
	 * Boolean param was added to avoid confusion with the constructor that would transform a scoring pose from a branch.
	 * For example, this is used for our auto align to L1.
	 */
	public PIDToPoseCommand(Drive drive, Superstructure superstructure, Pose2d rawEndPose, boolean useRaw) {
		this(
                drive,
                superstructure,
				rawEndPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)),
				SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(),
				SuperstructureConstants.getAutoAlignScoringAngleEpsilon(),
				SuperstructureConstants.getAutoAlignScoringDelay(),
				rawEndPose.getRotation(),
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

    public PIDToPoseCommand(
            Drive drive,
            Superstructure superstructure,
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(
                drive,
                superstructure,
				finalPose,
				epsilonDist,
				epsilonAngle,
				Units.Seconds.of(0.0),
				finalPose.getRotation(),
				translationController,
				headingController);
	}

    public PIDToPoseCommand(
			Drive drive, Superstructure superstructure, Pose2d finalPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		this(drive, superstructure, finalPose, Units.Seconds.of(0.00), translationController, headingController);
	}

	public PIDToPoseCommand(Drive drive, Superstructure superstructure, Pose2d finalPose) {
		this(drive, superstructure, finalPose, DriveConstants.mAutoAlignTranslationController, DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(Drive drive, Superstructure superstructure, Pose2d finalPose, Distance epsilonDist, Angle epsilonAngle) {
		this(drive, superstructure, finalPose, epsilonDist, epsilonAngle, DriveConstants.mAutoAlignTranslationController, DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(Drive drive, Superstructure superstructure, Pose2d finalPose, Distance epsilonDist, Angle epsilonAngle, SynchronousPIDF translationController) {
		this(drive, superstructure, finalPose, epsilonDist, epsilonAngle, translationController, DriveConstants.mAutoAlignHeadingController);
	}

    /* AUTO ALIGN PID TO POSE COMMANDS */
	

    public PIDToPoseCommand(Drive drive, Superstructure superstructure, Pose2d finalPose, SynchronousPIDF translationController) {
		this(drive, superstructure, finalPose, translationController, DriveConstants.mAutoAlignHeadingController);
	}

    @Override
	public void initialize() {
		if (superstructure != null) {
            superstructure.setPathFollowing(true);
        }
	}

    @Override
	public void execute() {
		LogUtil.recordPose2d("Auto Align PID/Final Pose", finalPose);
		drive.getDrivetrain().setControl(
				DriveConstants.getPIDToPoseRequestUpdater(drive, finalPose, translationController, headingController)
						.apply(DriveConstants.PIDToPoseRequest));
	}

    @Override
	public void end(boolean interrupted) {
		superstructure.setPathFollowing(false);
		drive.getDrivetrain().setControl(new SwerveRequest.ApplyRobotSpeeds());
	}

    @Override
	public boolean isFinished() {
		return atEndPose();
	}

    public boolean atEndPose() {
		Pose2d currentPose = drive.getPose();

		boolean complete = atTarget.update(
				Timer.getFPGATimestamp(),
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters)
						&& MathUtil.angleModulus(Math.abs(currentPose
										.getRotation()
										.minus(finalPose.getRotation())
										.getRadians()))
								< epsilonAngle.in(Units.Radians));

		SmartDashboard.putBoolean("Auto Align PID/Completed", complete);
		SmartDashboard.putBoolean(
				"Auto Align PID/Translation Completed",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters));
		SmartDashboard.putBoolean(
				"Auto Align PID/Rotation Completed",
				MathUtil.angleModulus(Math.abs(currentPose
								.getRotation()
								.minus(finalPose.getRotation())
								.getRadians()))
						< epsilonAngle.in(Units.Radians));
		SmartDashboard.putNumber(
				"Auto Align PID/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return complete;
	}

    public Distance distanceFromEnd() {
		return Units.Meters.of(drive
				.getPose()
				.getTranslation()
				.minus(finalPose.getTranslation())
				.getNorm());
	}

    public Superstructure getSuperstructure() {
        return superstructure;
    }
}
