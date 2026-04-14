package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in inches, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Inches</b> <br> as used in the game manual and field drawings
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public static AprilTagFieldLayout kAprilTagMap =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
	public static Distance kAprilTagWidth = Units.Inches.of(6.5);
    public static Distance kFieldLength = Units.Inches.of(651.22);
	public static Distance kFieldWidth = Units.Inches.of(317.69);
	public static Distance kAllianceZoneX = Units.Inches.of(158.61);
	public static Distance oppAllianceZone = Units.Meters.of(kAprilTagMap.getTagPose(10).get().getX());
    public static final Translation2d center = new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0));

    public static final Translation2d blueHubCenter = new Translation2d(Units.Inches.of(182.11), kFieldWidth.div(2.0));
	public static final Translation2d redHubCenter = mirrorAboutX(blueHubCenter, center.getMeasureX());
	public static final Pose2d blueOutpostPose = kAprilTagMap.getTagPose(13).get().toPose2d();
	public static final Translation2d blueDepotCenter = new Translation2d(Units.Inches.of(13.5), kFieldWidth.div(2.0).plus(Units.Inches.of(75.93)));
	public static final Translation2d kInnerBottomTrenchEdge = new Translation2d(Units.Inches.of(blueHubCenter.getX()), Units.Inches.of(50.34));
	public static final Translation2d kInnerTopTrenchEdge = new Translation2d(Units.Inches.of(blueHubCenter.getX()), kFieldWidth.minus(Units.Inches.of(50.34)));
	public static final Pose2d leftTower = kAprilTagMap.getTagPose(31).get().toPose2d().transformBy(new Transform2d(Units.Inches.of(40 + (3.51/2)), Units.Inches.of((32.25 / 2) + (5.875 + 1.5)), new Rotation2d()));
	public static final Pose2d rightTower = kAprilTagMap.getTagPose(31).get().toPose2d().transformBy(new Transform2d(Units.Inches.of(40 + (3.51/2)), Units.Inches.of((32.25 / 2) + (5.875 + 1.5)).unaryMinus(), new Rotation2d()));
	public static final Pose2d towerCenter = kAprilTagMap.getTagPose(31).get().toPose2d().transformBy(new Transform2d(Units.Inches.of(40 + (3.51/2)), Units.Inches.of(0.0), new Rotation2d()));

	public static final Distance neutralZoneNear = center.getMeasureY().minus(Units.Inches.of(120.0));
    public static final Distance neutralZoneFar = center.getMeasureY().plus(Units.Inches.of(120.0));

	public static final Distance rightBumpStart = Units.Inches.of(158.84 - (47 / 2));
	public static final Distance leftBumpEnd = Units.Inches.of(158.84 + (47 / 2));

	public static final Distance trenchTagToCenter = Units.Inches.of(1.5/2.0);

	public static final Translation2d kLeftBlueTrench = new Translation2d(blueHubCenter.getMeasureX(), kAprilTagMap.getTagPose(23).get().getMeasureY().plus(trenchTagToCenter));
	public static final Translation2d kRightBlueTrench = new Translation2d(blueHubCenter.getMeasureX(), kAprilTagMap.getTagPose(28).get().getMeasureY().plus(trenchTagToCenter));
	public static final Translation2d kLeftRedTrench = new Translation2d(redHubCenter.getMeasureX(), kAprilTagMap.getTagPose(7).get().getMeasureY().minus(trenchTagToCenter));
	public static final Translation2d kRightRedTrench = new Translation2d(redHubCenter.getMeasureX(), kAprilTagMap.getTagPose(12).get().getMeasureY().minus(trenchTagToCenter));


	public static final Rectangle2d rightNeutralZone = new Rectangle2d(
		handleAllianceFlip(new Translation2d(edu.wpi.first.math.util.Units.inchesToMeters(205.61), 0), RobotConstants.isRedAlliance),
		handleAllianceFlip(new Translation2d(edu.wpi.first.math.util.Units.inchesToMeters(325.61), edu.wpi.first.math.util.Units.inchesToMeters(158.84)), RobotConstants.isRedAlliance)
	);

	public static final Rectangle2d leftNeutralZone = new Rectangle2d(
		handleAllianceFlip(new Translation2d(edu.wpi.first.math.util.Units.inchesToMeters(205.61), edu.wpi.first.math.util.Units.inchesToMeters(158.84)), RobotConstants.isRedAlliance),
		handleAllianceFlip(new Translation2d(edu.wpi.first.math.util.Units.inchesToMeters(325.61), edu.wpi.first.math.util.Units.inchesToMeters(317.69)), RobotConstants.isRedAlliance)
	);
	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = rotateAboutCenter(blue_pose, Rotation2d.k180deg);
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.rotateAround(
					new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), Rotation2d.k180deg);
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.plus(Rotation2d.k180deg);
		}
		return blue_rotation;
	}

	public static Rectangle2d handleAllianceFlip(Rectangle2d rect, boolean is_red_alliance) {
		Translation2d cornerA = new Translation2d(
			rect.getCenter().getTranslation().getX() - rect.getXWidth()/2,
			rect.getCenter().getTranslation().getY() - rect.getYWidth()/2
		);
		Translation2d cornerB = new Translation2d(
			rect.getCenter().getTranslation().getX() + rect.getXWidth()/2,
			rect.getCenter().getTranslation().getY() + rect.getYWidth()/2
		);

		return new Rectangle2d(handleAllianceFlip(cornerA, is_red_alliance), handleAllianceFlip(cornerB, is_red_alliance));
	}

	public static Distance distanceFromAllianceWall(Distance x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength.minus(x_coordinate);
		}
		return x_coordinate;
	}

	public static Distance calculateRetractZoneWidth(
			Pose2d robotPose,
			ChassisSpeeds robotSpeeds,
			Translation2d trenchCenter) {

		Translation2d fieldVelocity =
				new Translation2d(
						robotSpeeds.vxMetersPerSecond,
						robotSpeeds.vyMetersPerSecond);

		Translation2d toTrench =
				trenchCenter.minus(robotPose.getTranslation());

		Translation2d directionToTrench =
				toTrench.div(toTrench.getNorm());

		double velocityTowardTrench =
				fieldVelocity.getX() * directionToTrench.getX()
			+ fieldVelocity.getY() * directionToTrench.getY();

		velocityTowardTrench = Math.max(0.0, velocityTowardTrench);

		LinearVelocity vToward =
				Units.MetersPerSecond.of(velocityTowardTrench);

		LinearVelocity velocityAtRetract =
				Units.MetersPerSecond.of(
						Math.min(
								vToward
									.plus(DriveConstants.kMaxAcceleration
											.times(HoodConstants.retractTime))
									.in(Units.MetersPerSecond),
								TunerConstants.kSpeedAt12Volts
									.in(Units.MetersPerSecond)));

		Distance zoneWidth = Units.Inches.of(47);
		zoneWidth =
				zoneWidth.plus(
						HoodConstants.retractTime.times(velocityAtRetract));

		return zoneWidth;
	}



	public static boolean nearTrench(
			Pose2d pose,
			ChassisSpeeds currentSpeeds) {

		Pose2d shooterPose = pose.plus(ShooterConstants.robotToShooter);
		
		Distance x = shooterPose.getMeasureX();
		Distance y = shooterPose.getMeasureY();

		boolean inTrenchY =
				y.lte(kInnerBottomTrenchEdge.getMeasureY())
			|| y.gte(kInnerTopTrenchEdge.getMeasureY());

		if (!inTrenchY) {
			return false;
		}

		double fieldVx = currentSpeeds.vxMetersPerSecond;

		Distance[] trenchCenters = {
			kLeftBlueTrench.getMeasureX(),
			kRightBlueTrench.getMeasureX(),
			kLeftRedTrench.getMeasureX(),
			kRightRedTrench.getMeasureX()
		};

		for (Distance trenchX : trenchCenters) {

			double direction =
					Math.signum(
						trenchX.minus(x).in(Units.Meters));

			// Only count velocity toward trench
			double velocityToward =
					Math.max(0.0, fieldVx * direction);

			LinearVelocity vToward =
					Units.MetersPerSecond.of(velocityToward);

			LinearVelocity velocityAtRetract =
					Units.MetersPerSecond.of(
							Math.min(
									vToward
										.plus(DriveConstants.kMaxAcceleration
												.times(HoodConstants.retractTime))
										.in(Units.MetersPerSecond),
									TunerConstants.kSpeedAt12Volts
										.in(Units.MetersPerSecond)));

			Distance retractZoneWidth =
					Units.Inches.of(47)
						.plus(HoodConstants.retractTime
								.times(velocityAtRetract));

			if (x.isNear(trenchX, retractZoneWidth)) {
				return true;
			}
		}

		return false;
	}


	/** Changes heading so the robot doesn't get stuck in the trench at certain angles
	 * @param rotation current rotation
	 * @return adjusted rotation
	 */
	public static Rotation2d clampAwayFromTrench(Rotation2d rotation) {
		double deg = rotation.getDegrees();

		if (deg >= 30 && deg <= 80) {
			return Rotation2d.fromDegrees((deg - 30 < 80 - deg) ? 30 : 80);
		}

		if (deg >= -150 && deg <= -100) {
			return Rotation2d.fromDegrees((deg - (-150) < -100 - deg) ? -150 : -100);
		}

		if (deg >= 120 && deg <= 170) {
			return Rotation2d.fromDegrees((deg - 120 < 170 - deg) ? 120 : 170);
		}

		if (deg >= -60 && deg <= -10) {
			return Rotation2d.fromDegrees((deg - (-60) < -10 - deg) ? -60 : -10);
		}

		return rotation;
	}





	public static Translation2d mirrorAboutX(Translation2d t, Distance xValue) {
		return new Translation2d(xValue.in(Units.Meters) + (xValue.in(Units.Meters) - t.getX()), t.getY());
	}

	public static Translation2d mirrorAboutY(Translation2d t, Distance yValue) {
		return new Translation2d(t.getX(), yValue.in(Units.Meters) + (yValue.in(Units.Meters) - t.getY()));
	}

	public static Rotation2d mirrorAboutX(Rotation2d r) {
		return new Rotation2d(-r.getCos(), r.getSin());
	}

	public static Rotation2d mirrorAboutY(Rotation2d r) {
		return new Rotation2d(r.getCos(), -r.getSin());
	}

	public static Pose2d mirrorAboutX(Pose2d p, Distance xValue) {
		return new Pose2d(mirrorAboutX(p.getTranslation(), xValue), mirrorAboutX(p.getRotation()));
	}

	public static Pose2d mirrorAboutY(Pose2d p, Distance yValue) {
		return new Pose2d(mirrorAboutY(p.getTranslation(), yValue), mirrorAboutY(p.getRotation()));
	}

	public static Pose2d rotateAboutPose(Pose2d startPose, Translation2d point, Rotation2d rotation) {
		return new Pose2d(
				startPose.getTranslation().rotateAround(point, rotation),
				startPose.getRotation().plus(rotation));
	}

	public static Pose2d rotateAboutCenter(Pose2d startPose, Rotation2d rotation) {
		return rotateAboutPose(startPose, new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), rotation);
	}

}