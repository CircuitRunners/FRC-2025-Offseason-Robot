package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Util {
	public static final double kEpsilon = 1e-12;

	/**
	 * Prevent this class from being instantiated.
	 */
	private Util() {}
	
    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static int limit(int v, int min, int max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

	public static Rotation2d flipRedBlue(Rotation2d original) {
        return new Rotation2d(original.getRadians() + Math.PI);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, kEpsilon);
	}

	public static boolean epsilonEquals(int a, int b, int epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean epsilonEquals(Translation2d a, Translation2d b) {
		return epsilonEquals(a.getX(), b.getX()) || epsilonEquals(a.getY(), b.getY());
	}

	public static boolean epsilonEquals(Translation2d a, Translation2d b, double epsilon) {
		return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
	}

	public static boolean epsilonEquals(Translation2d a, Translation2d b, Distance epsilon) {
		return epsilonEquals(a.getX(), b.getX(), epsilon.in(Units.Meters))
				&& epsilonEquals(a.getY(), b.getY(), epsilon.in(Units.Meters));
	}

	public static boolean epsilonEquals(ChassisSpeeds a, ChassisSpeeds b) {
		return epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond)
				&& epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond)
				&& epsilonEquals(a.omegaRadiansPerSecond, b.omegaRadiansPerSecond);
	}

	public static boolean epsilonEquals(ChassisSpeeds a, ChassisSpeeds b, double linearVelocityEpsilon) {
		return epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond, linearVelocityEpsilon)
				&& epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond, linearVelocityEpsilon);
	}

	public static class DistanceAngleConverter {
		private final Distance radius;

		public DistanceAngleConverter(Distance radius) {
			this.radius = radius;
		}

		/**
		 * Converts a distance measurement to an equal angle measurement based on radius initialized with.
		 *
		 * @param distance Distance to convert to angle.
		 * @return Angle distance is equivalent to.
		 */
		public Angle toAngle(Distance distance) {
			return Units.Radians.of(distance.in(BaseUnits.DistanceUnit) / radius.baseUnitMagnitude());
		}

		/**
		 * Converts an angle measurement to an equal distance measurement based on radius initialized with.
		 *
		 * @param distance angle to convert to distance.
		 * @return Distance agle is equivalent to.
		 */
		public Distance toDistance(Angle angle) {
			return BaseUnits.DistanceUnit.of(angle.in(Units.Radians) * radius.baseUnitMagnitude());
		}

		/**
		 * Gets an angle unit equivalent to a distance unit with the conversion of the radius initialized with.
		 *
		 * @param unit The distance unit to convert.
		 * @return The distance represented as an AngleUnit
		 */
		public AngleUnit getDistanceUnitAsAngleUnit(DistanceUnit unit) {
			return Units.derive(BaseUnits.AngleUnit)
					.aggregate(toAngle(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		/**
		 * Gets a distance unit equivalent to a angle unit with the conversion of the radius initialized with.
		 *
		 * @param unit The angle unit to convert.
		 * @return The distance represented as a DistanceUnit
		 */
		public DistanceUnit getAngleUnitAsDistanceUnit(AngleUnit unit) {
			return Units.derive(BaseUnits.DistanceUnit)
					.splitInto(toDistance(unit.one()).baseUnitMagnitude())
					.named(unit.name())
					.symbol(unit.symbol())
					.make();
		}

		public Distance getDrumRadius() {
			return radius;
		}
	}

    public static class Pose2dTimeInterpolable {
		private List<Pair<Pose2d, Time>> poseList = new ArrayList<>();

		public Time getTimeFromPose(Pose2d pose) {
			Pair<Pose2d, Time> prevState = poseList.get(0);
			Pair<Pose2d, Time> nextState = poseList.get(0);
			for (int i = 0; i < poseList.size() - 1; i++) {
				if (i >= poseList.size() - 2) {
					return poseList.get(poseList.size() - 1).getSecond();
				}
				prevState = poseList.get(i);
				nextState = poseList.get(i + 2);
				if (prevState.getFirst().getTranslation().getDistance(pose.getTranslation())
						< nextState.getFirst().getTranslation().getDistance(pose.getTranslation())) {
					nextState = poseList.get(i + 1);
					break;
				}
			}

			double distanceToPrevPose = prevState.getFirst().getTranslation().getDistance(pose.getTranslation());
			double distanceToNextPose = nextState.getFirst().getTranslation().getDistance(pose.getTranslation());
			double percentToNextPose = (prevState
									.getFirst()
									.getTranslation()
									.getDistance(nextState.getFirst().getTranslation())
							- distanceToNextPose)
					/ (distanceToPrevPose + distanceToNextPose);

			Time timeDelta = nextState.getSecond().minus(prevState.getSecond());
			Time timeAtPose = prevState.getSecond().plus(timeDelta.times(percentToNextPose));
			return timeAtPose;
		}

		public Pose2d getPoseFromTime(Time time) {
			if (time.gte(poseList.get(poseList.size() - 1).getSecond())) {
				return poseList.get(poseList.size() - 1).getFirst();
			} else if (time.lte(poseList.get(0).getSecond())) {
				return poseList.get(0).getFirst();
			}

			Pair<Pose2d, Time> prevState = poseList.get(0);
			Pair<Pose2d, Time> nextState = poseList.get(0);
			;

			for (int i = 1; i < poseList.size(); i++) {
				nextState = poseList.get(i);
				if (nextState.getSecond().gte(time)) {
					prevState = poseList.get(i - 1);
					break;
				}
			}

			Time timeDelta = nextState.getSecond().minus(prevState.getSecond());
			double percentIntoDelta =
					time.minus(prevState.getSecond()).in(BaseUnits.TimeUnit) / (timeDelta.in(BaseUnits.TimeUnit));
			Transform2d prevToTimePose =
					nextState.getFirst().minus(prevState.getFirst()).times(percentIntoDelta);
			return prevState.getFirst().plus(prevToTimePose);
		}

		public void clearStatesBeforeTime(Time time) {
			while (poseList.size() > 1 && poseList.get(0).getSecond().lt(time)) {
				poseList.remove(0);
			}
		}

		public Pose2dTimeInterpolable(Trajectory trajwithTan, Rotation2d startHeading, Rotation2d endHeading) {
			double totalTimeSeconds = trajwithTan.getTotalTimeSeconds();
			for (State state : trajwithTan.getStates()) {
				Rotation2d poseRotation = startHeading.interpolate(endHeading, state.timeSeconds / totalTimeSeconds);
				poseList.add(new Pair<>(
						new Pose2d(state.poseMeters.getTranslation(), poseRotation),
						Units.Seconds.of(state.timeSeconds)));
			}
			SmartDashboard.putNumber("Auto Align Traj/Number Of Trajectory States", poseList.size());
		}
	}

	public static double calculateDistanceToStartDeccel(
			double currentVel, double stowedAccel, double raisedAccel, double timeToRaise) {
		double distToRaiseElev = calculatDistanceToRaiseElevator(raisedAccel, timeToRaise);
		double velAtDistToRaiseElev = raisedAccel * timeToRaise;
		double deltaVel = currentVel - velAtDistToRaiseElev;
		double timeToDeccelDelta = deltaVel / stowedAccel;
		double distToDeccelDelta = 0.5 * stowedAccel * timeToDeccelDelta * timeToDeccelDelta;
		double totalDist = distToDeccelDelta + distToRaiseElev;
		return totalDist;
	}

	public static double calculatDistanceToRaiseElevator(double raisedAccel, double timeToRaise) {
		return 0.5 * raisedAccel * timeToRaise * timeToRaise;
	}
}
