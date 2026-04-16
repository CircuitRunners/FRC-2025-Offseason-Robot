package frc.lib.drive;

import java.util.List;

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

public class PIDToPosesCommand extends Command {
    Drive drive;
    Superstructure superstructure;
    List<Pose2d> finalPose;
    Distance epsilonDist;
    Angle epsilonAngle;
    Util.Pose2dTimeInterpolable interpolable;
    DelayedBoolean atTarget;
    boolean isAuto;
    SynchronousPIDF translationController;
    SynchronousPIDF headingController;
    int currentIndex;
    Pose2d currentTarget;
	
    public PIDToPosesCommand(
            Drive drive,
            Superstructure superstructure,
            List<Pose2d> finalPose,
            Distance epsilonDist,
            Angle epsilonAngle,
            Time delayTime,
            SynchronousPIDF translationController,
            SynchronousPIDF headingController) {
        addRequirements(drive);

        this.drive = drive;
        this.superstructure = superstructure;
        this.finalPose = finalPose;
		this.epsilonDist = epsilonDist;
		this.epsilonAngle = epsilonAngle;
		this.translationController = translationController;
		this.headingController = headingController;

        atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));
    }



    public PIDToPosesCommand(
            Drive drive,
            Superstructure superstructure,
            List<Pose2d> finalPose,
            Time delayTime,
            SynchronousPIDF translationController,
            SynchronousPIDF headingController) {
        this(
                drive,
                superstructure,
                finalPose,
                Units.Centimeters.of(10),
                Units.Degrees.of(.8),
                delayTime,
                translationController,
                headingController);
    }




    public PIDToPosesCommand(
            Drive drive,
            Superstructure superstructure,
			List<Pose2d> finalPose,
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
				translationController,
				headingController);
	}

    public PIDToPosesCommand(
			Drive drive, Superstructure superstructure, List<Pose2d> finalPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		this(drive, superstructure, finalPose, Units.Seconds.of(0.00), translationController, headingController);
	}

    /* AUTO ALIGN PID TO POSE COMMANDS */
	

    public PIDToPosesCommand(Drive drive, Superstructure superstructure, List<Pose2d> finalPose, SynchronousPIDF translationController) {
		this(drive, superstructure, finalPose, translationController, DriveConstants.mAutoAlignHeadingController);
	}

    @Override
	public void initialize() {
        currentIndex = 0;
        currentTarget = finalPose.get(currentIndex);
		if (superstructure != null) {
            superstructure.setPathFollowing(true);
        }
	}


    @Override
    public void execute() {

        Pose2d currentPose = drive.getPose();

        Translation2d toTarget =
            currentTarget.getTranslation().minus(currentPose.getTranslation());


        double distanceToTarget =
            currentPose.getTranslation()
                .getDistance(currentTarget.getTranslation());
        double switchDistance = 0.25;

        if (distanceToTarget < switchDistance &&
            currentIndex < finalPose.size() - 1) {

            currentIndex++;
            currentTarget = finalPose.get(currentIndex);
        }

        Rotation2d approachHeading =
            new Rotation2d(toTarget.getX(), toTarget.getY());

        double headingError =
            MathUtil.angleModulus(
                approachHeading.minus(currentPose.getRotation()).getRadians());

        double farThreshold = Math.toRadians(35);
        double nearThreshold = Math.toRadians(12);
        double thresholdRadius = 0.6;

        double t = MathUtil.clamp(distanceToTarget / thresholdRadius, 0.0, 1.0);
        double rotationLockThreshold =
            nearThreshold + (farThreshold - nearThreshold) * t;

        Pose2d dynamicTarget;

        boolean isLast = (currentIndex == finalPose.size() - 1);

        if (isLast && distanceToTarget < 0.35) {
            dynamicTarget = currentTarget;
        }
        else if (Math.abs(headingError) > rotationLockThreshold) {

            dynamicTarget = new Pose2d(
                currentPose.getTranslation(),
                approachHeading
            );

        } 
        else {
            dynamicTarget = new Pose2d(
                currentTarget.getTranslation(),
                approachHeading
            );
        }

        drive.getDrivetrain().setControl(
            DriveConstants.getPIDToPoseRequestUpdater(
                drive,
                dynamicTarget,
                translationController,
                headingController)
            .apply(DriveConstants.PIDToPoseRequest)
        );
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setPathFollowing(false);

        if (interrupted) {
            drive.getDrivetrain().setControl(new SwerveRequest.ApplyRobotSpeeds());
        }
    }

    @Override
	public boolean isFinished() {
		return atEndPose();
	}

    public boolean atEndPose() {
		Pose2d currentPose = drive.getPose();

		boolean complete = (currentIndex == finalPose.size()-1) && atTarget.update(
				Timer.getFPGATimestamp(),
				currentPose.getTranslation().getDistance(finalPose.get(finalPose.size()-1).getTranslation()) < epsilonDist.in(Units.Meters)
						&& MathUtil.angleModulus(Math.abs(currentPose
										.getRotation()
										.minus(finalPose.get(finalPose.size() -1).getRotation())
										.getRadians()))
								< epsilonAngle.in(Units.Radians));

		SmartDashboard.putBoolean("Auto Align PID/Completed", complete);
		SmartDashboard.putBoolean(
				"Auto Align PID/Translation Completed",
				currentPose.getTranslation().getDistance(finalPose.get(finalPose.size()-1).getTranslation()) < epsilonDist.in(Units.Meters));
		SmartDashboard.putBoolean(
				"Auto Align PID/Rotation Completed",
				MathUtil.angleModulus(Math.abs(currentPose
								.getRotation()
								.minus(finalPose.get(finalPose.size()-1).getRotation())
								.getRadians()))
						< epsilonAngle.in(Units.Radians));
		SmartDashboard.putNumber(
				"Auto Align PID/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.get(finalPose.size()-1).getTranslation()) * 39.37);

		return complete;
	}

    public Distance distanceFromEnd() {
		return Units.Meters.of(drive
				.getPose()
				.getTranslation()
				.minus(finalPose.get(finalPose.size()-1).getTranslation())
				.getNorm());
	}

    public Superstructure getSuperstructure() {
        return superstructure;
    }
}
