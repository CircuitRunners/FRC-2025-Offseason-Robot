package frc.robot.subsystems.vision.objectdetection;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollerConstants;

public class FuelPathFinder {
    private static final double HEADING_STEP_DEGREES = 15.0;
    private static final double LANE_SPACING_METERS = IntakeRollerConstants.intakeWidth * 0.8;
    private static final double CLUSTER_LINK_METERS = 0.95;
    private static final double ENTRY_LEAD_METERS = 0.35;
    private static final double EXIT_FOLLOW_THROUGH_METERS = 0.20;
    private static final double MIN_POSE_SPACING_METERS = 0.05;

    public FuelPathPlan findPath(Pose2d robotPose, List<Translation2d> fuelPositions) {
        return findPath(robotPose, fuelPositions, IntakeRollerConstants.fuelLimit);
    }

    public FuelPathPlan findPath(Pose2d robotPose, List<Translation2d> fuelPositions, int maxFuelToCollect) {
        if (robotPose == null || fuelPositions == null || fuelPositions.isEmpty() || maxFuelToCollect <= 0) {
            return FuelPathPlan.empty();
        }

        FuelPathPlan bestPlan = FuelPathPlan.empty();
        for (List<Translation2d> cluster : clusterFuel(fuelPositions)) {
            if (cluster.isEmpty()) {
                continue;
            }

            for (Rotation2d sweepHeading : getCandidateSweepHeadings(robotPose, cluster)) {
                FuelPathPlan candidatePlan = findBestSweepForHeading(robotPose, cluster, sweepHeading, maxFuelToCollect);
                if (candidatePlan.score() > bestPlan.score()) {
                    bestPlan = candidatePlan;
                }
            }
        }

        return bestPlan;
    }

    public Trajectory buildTrajectory(Pose2d startPose, FuelPathPlan plan) {
        return buildTrajectory(
                startPose,
                plan,
                new TrajectoryConfig(DriveConstants.kIntakeMaxSpeed, DriveConstants.kMaxAcceleration));
    }

    public Trajectory buildTrajectory(Pose2d startPose, FuelPathPlan plan, TrajectoryConfig config) {
        if (startPose == null || plan == null || plan.isEmpty()) {
            return new Trajectory();
        }

        ArrayList<Pose2d> allPoses = new ArrayList<>();
        allPoses.add(startPose);
        allPoses.addAll(plan.pathPoses());

        Trajectory combinedTrajectory = null;
        for (int i = 0; i < allPoses.size() - 1; i++) {
            Pose2d start = allPoses.get(i);
            Pose2d end = allPoses.get(i + 1);
            if (start.getTranslation().getDistance(end.getTranslation()) < MIN_POSE_SPACING_METERS) {
                continue;
            }

            Trajectory segment = TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
            combinedTrajectory = combinedTrajectory == null ? segment : combinedTrajectory.concatenate(segment);
        }

        return combinedTrajectory == null ? new Trajectory() : combinedTrajectory;
    }

    private FuelPathPlan findBestSweepForHeading(
            Pose2d robotPose,
            List<Translation2d> cluster,
            Rotation2d sweepHeading,
            int maxFuelToCollect) {
        List<SweepLane> lanes = buildSweepLanes(cluster, sweepHeading);
        if (lanes.isEmpty()) {
            return FuelPathPlan.empty();
        }

        FuelPathPlan bestPlan = FuelPathPlan.empty();
        for (int startIndex = 0; startIndex < lanes.size(); startIndex++) {
            for (int endIndex = startIndex; endIndex < lanes.size(); endIndex++) {
                List<SweepLane> ascendingWindow = new ArrayList<>(lanes.subList(startIndex, endIndex + 1));
                List<SweepLane> descendingWindow = new ArrayList<>(ascendingWindow);
                Collections.reverse(descendingWindow);

                bestPlan = chooseBetter(
                        bestPlan,
                        buildLaneTraversalPlan(robotPose, ascendingWindow, sweepHeading, true, maxFuelToCollect));
                bestPlan = chooseBetter(
                        bestPlan,
                        buildLaneTraversalPlan(robotPose, ascendingWindow, sweepHeading, false, maxFuelToCollect));
                bestPlan = chooseBetter(
                        bestPlan,
                        buildLaneTraversalPlan(robotPose, descendingWindow, sweepHeading, true, maxFuelToCollect));
                bestPlan = chooseBetter(
                        bestPlan,
                        buildLaneTraversalPlan(robotPose, descendingWindow, sweepHeading, false, maxFuelToCollect));
            }
        }

        return bestPlan;
    }

    private FuelPathPlan buildLaneTraversalPlan(
            Pose2d robotPose,
            List<SweepLane> laneOrder,
            Rotation2d sweepHeading,
            boolean startMovingPositive,
            int maxFuelToCollect) {
        if (laneOrder.isEmpty() || maxFuelToCollect <= 0) {
            return FuelPathPlan.empty();
        }

        ArrayList<Pose2d> pathPoses = new ArrayList<>();
        ArrayList<Translation2d> collectedFuel = new ArrayList<>();
        Translation2d currentPosition = robotPose.getTranslation();
        boolean travelPositive = startMovingPositive;
        double totalTravelDistance = 0.0;
        int usedLanes = 0;
        int remainingFuelCapacity = maxFuelToCollect;

        for (SweepLane lane : laneOrder) {
            if (remainingFuelCapacity <= 0) {
                break;
            }

            LaneTraversal traversal = lane.createTraversal(travelPositive, remainingFuelCapacity, sweepHeading);
            if (traversal == null || traversal.collectedFuel.isEmpty()) {
                travelPositive = !travelPositive;
                continue;
            }

            usedLanes++;
            totalTravelDistance += currentPosition.getDistance(traversal.entryPose.getTranslation());
            addPoseIfDistinct(pathPoses, traversal.entryPose);

            if (traversal.entryPose.getTranslation().getDistance(traversal.exitPose.getTranslation()) >= MIN_POSE_SPACING_METERS) {
                totalTravelDistance += traversal.entryPose.getTranslation().getDistance(traversal.exitPose.getTranslation());
                addPoseIfDistinct(pathPoses, traversal.exitPose);
            }

            currentPosition = traversal.exitPose.getTranslation();
            collectedFuel.addAll(traversal.collectedFuel);
            remainingFuelCapacity -= traversal.collectedFuel.size();
            travelPositive = !travelPositive;
        }

        if (pathPoses.isEmpty() || collectedFuel.isEmpty()) {
            return FuelPathPlan.empty();
        }

        Pose2d firstPose = pathPoses.get(0);
        double initialTurnPenalty =
                Math.abs(MathUtil.angleModulus(firstPose.getRotation().minus(robotPose.getRotation()).getRadians()));
        double score =
                collectedFuel.size() * 6.0
                        + (collectedFuel.size() / Math.max(totalTravelDistance, 0.35)) * 2.0
                        - totalTravelDistance * 1.35
                        - Math.max(0, usedLanes - 1) * 0.75
                        - initialTurnPenalty * 0.6;

        return new FuelPathPlan(pathPoses, collectedFuel, score, totalTravelDistance);
    }

    private List<SweepLane> buildSweepLanes(List<Translation2d> fuelPositions, Rotation2d sweepHeading) {
        Translation2d forwardUnit = unitFromRotation(sweepHeading);
        Translation2d leftUnit = leftUnitFromRotation(sweepHeading);

        ArrayList<ProjectedFuel> projectedFuel = new ArrayList<>();
        double minLateral = Double.POSITIVE_INFINITY;
        for (Translation2d fuel : fuelPositions) {
            double forward = dot(fuel, forwardUnit);
            double lateral = dot(fuel, leftUnit);
            projectedFuel.add(new ProjectedFuel(fuel, forward, lateral));
            minLateral = Math.min(minLateral, lateral);
        }

        Map<Integer, ArrayList<ProjectedFuel>> lanesByIndex = new HashMap<>();
        for (ProjectedFuel fuel : projectedFuel) {
            int laneIndex = (int) Math.round((fuel.lateral - minLateral) / LANE_SPACING_METERS);
            lanesByIndex.computeIfAbsent(laneIndex, ignored -> new ArrayList<>()).add(fuel);
        }

        ArrayList<SweepLane> lanes = new ArrayList<>();
        for (ArrayList<ProjectedFuel> laneFuel : lanesByIndex.values()) {
            laneFuel.sort(Comparator.comparingDouble(projected -> projected.forward));
            double averageLateral = 0.0;
            for (ProjectedFuel fuel : laneFuel) {
                averageLateral += fuel.lateral;
            }
            averageLateral /= laneFuel.size();
            lanes.add(new SweepLane(averageLateral, laneFuel));
        }

        lanes.sort(Comparator.comparingDouble(lane -> lane.lateral));
        return lanes;
    }

    private List<Rotation2d> getCandidateSweepHeadings(Pose2d robotPose, List<Translation2d> cluster) {
        Set<Integer> usedBuckets = new HashSet<>();
        ArrayList<Rotation2d> headings = new ArrayList<>();

        Rotation2d principalHeading = getPrincipalHeading(cluster);
        Rotation2d axisAlignedHeading = getAxisAlignedHeading(cluster);
        Rotation2d toCentroidHeading = clusterCenter(cluster).minus(robotPose.getTranslation()).getAngle();

        addHeading(headings, usedBuckets, principalHeading);
        addHeading(headings, usedBuckets, principalHeading.plus(Rotation2d.fromDegrees(90.0)));
        addHeading(headings, usedBuckets, axisAlignedHeading);
        addHeading(headings, usedBuckets, axisAlignedHeading.plus(Rotation2d.fromDegrees(90.0)));
        addHeading(headings, usedBuckets, toCentroidHeading);
        addHeading(headings, usedBuckets, toCentroidHeading.plus(Rotation2d.fromDegrees(90.0)));

        if (headings.isEmpty()) {
            headings.add(Rotation2d.kZero);
        }
        return headings;
    }

    private void addHeading(List<Rotation2d> headings, Set<Integer> usedBuckets, Rotation2d heading) {
        double normalizedDegrees = Math.toDegrees(normalizeSweepRadians(heading.getRadians()));
        double quantizedDegrees = Math.round(normalizedDegrees / HEADING_STEP_DEGREES) * HEADING_STEP_DEGREES;
        if (quantizedDegrees >= 180.0) {
            quantizedDegrees -= 180.0;
        }

        int key = (int) Math.round(quantizedDegrees * 1000.0);
        if (usedBuckets.add(key)) {
            headings.add(Rotation2d.fromDegrees(quantizedDegrees));
        }
    }

    private Rotation2d getPrincipalHeading(List<Translation2d> cluster) {
        Translation2d center = clusterCenter(cluster);
        double sxx = 0.0;
        double syy = 0.0;
        double sxy = 0.0;

        for (Translation2d fuel : cluster) {
            double dx = fuel.getX() - center.getX();
            double dy = fuel.getY() - center.getY();
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }

        double angleRadians = 0.5 * Math.atan2(2.0 * sxy, sxx - syy);
        return Rotation2d.fromRadians(angleRadians);
    }

    private Rotation2d getAxisAlignedHeading(List<Translation2d> cluster) {
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (Translation2d fuel : cluster) {
            minX = Math.min(minX, fuel.getX());
            maxX = Math.max(maxX, fuel.getX());
            minY = Math.min(minY, fuel.getY());
            maxY = Math.max(maxY, fuel.getY());
        }

        return (maxX - minX) >= (maxY - minY) ? Rotation2d.kZero : Rotation2d.fromDegrees(90.0);
    }

    private List<List<Translation2d>> clusterFuel(List<Translation2d> fuelPositions) {
        ArrayList<List<Translation2d>> clusters = new ArrayList<>();
        boolean[] visited = new boolean[fuelPositions.size()];

        for (int i = 0; i < fuelPositions.size(); i++) {
            if (visited[i]) {
                continue;
            }

            ArrayDeque<Integer> queue = new ArrayDeque<>();
            ArrayList<Translation2d> cluster = new ArrayList<>();
            queue.add(i);
            visited[i] = true;

            while (!queue.isEmpty()) {
                int currentIndex = queue.removeFirst();
                Translation2d currentFuel = fuelPositions.get(currentIndex);
                cluster.add(currentFuel);

                for (int otherIndex = 0; otherIndex < fuelPositions.size(); otherIndex++) {
                    if (visited[otherIndex]) {
                        continue;
                    }

                    if (currentFuel.getDistance(fuelPositions.get(otherIndex)) <= CLUSTER_LINK_METERS) {
                        visited[otherIndex] = true;
                        queue.addLast(otherIndex);
                    }
                }
            }

            clusters.add(cluster);
        }

        clusters.sort(Comparator.comparingInt(List<Translation2d>::size).reversed());
        return clusters;
    }

    private void addPoseIfDistinct(List<Pose2d> poses, Pose2d candidatePose) {
        if (poses.isEmpty()) {
            poses.add(candidatePose);
            return;
        }

        Pose2d previousPose = poses.get(poses.size() - 1);
        if (previousPose.getTranslation().getDistance(candidatePose.getTranslation()) < MIN_POSE_SPACING_METERS) {
            poses.set(poses.size() - 1, candidatePose);
            return;
        }

        poses.add(candidatePose);
    }

    private Translation2d clusterCenter(List<Translation2d> cluster) {
        double xSum = 0.0;
        double ySum = 0.0;
        for (Translation2d fuel : cluster) {
            xSum += fuel.getX();
            ySum += fuel.getY();
        }
        return new Translation2d(xSum / cluster.size(), ySum / cluster.size());
    }

    private Translation2d unitFromRotation(Rotation2d rotation) {
        return new Translation2d(rotation.getCos(), rotation.getSin());
    }

    private Translation2d leftUnitFromRotation(Rotation2d rotation) {
        return new Translation2d(-rotation.getSin(), rotation.getCos());
    }

    private double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    private double normalizeSweepRadians(double radians) {
        double normalized = MathUtil.angleModulus(radians);
        if (normalized < 0.0) {
            normalized += Math.PI;
        }
        if (normalized >= Math.PI) {
            normalized -= Math.PI;
        }
        return normalized;
    }

    private FuelPathPlan chooseBetter(FuelPathPlan currentBest, FuelPathPlan candidate) {
        return candidate.score() > currentBest.score() ? candidate : currentBest;
    }

    private Translation2d pointFromProjection(double forward, double lateral, Rotation2d sweepHeading) {
        Translation2d forwardUnit = unitFromRotation(sweepHeading);
        Translation2d leftUnit = leftUnitFromRotation(sweepHeading);
        return new Translation2d(
                forward * forwardUnit.getX() + lateral * leftUnit.getX(),
                forward * forwardUnit.getY() + lateral * leftUnit.getY());
    }

    public static record FuelPathPlan(
            List<Pose2d> pathPoses,
            List<Translation2d> collectedFuel,
            double score,
            double pathLengthMeters) {
        public FuelPathPlan {
            pathPoses = List.copyOf(pathPoses);
            collectedFuel = List.copyOf(collectedFuel);
        }

        public static FuelPathPlan empty() {
            return new FuelPathPlan(List.of(), List.of(), Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        public boolean isEmpty() {
            return pathPoses.isEmpty();
        }
    }

    private class SweepLane {
        private final double lateral;
        private final List<ProjectedFuel> fuel;

        private SweepLane(double lateral, List<ProjectedFuel> fuel) {
            this.lateral = lateral;
            this.fuel = fuel;
        }

        private LaneTraversal createTraversal(boolean positiveDirection, int maxFuelToCollect, Rotation2d sweepHeading) {
            if (fuel.isEmpty() || maxFuelToCollect <= 0) {
                return null;
            }

            int fuelToTake = Math.min(maxFuelToCollect, fuel.size());
            ArrayList<Translation2d> collectedFuel = new ArrayList<>();
            Rotation2d laneHeading = positiveDirection ? sweepHeading : sweepHeading.plus(Rotation2d.k180deg);

            double startForward;
            double endForward;
            if (positiveDirection) {
                startForward = fuel.get(0).forward - ENTRY_LEAD_METERS;
                endForward = fuel.get(fuelToTake - 1).forward + EXIT_FOLLOW_THROUGH_METERS;
                for (int i = 0; i < fuelToTake; i++) {
                    collectedFuel.add(fuel.get(i).position);
                }
            } else {
                startForward = fuel.get(fuel.size() - 1).forward + ENTRY_LEAD_METERS;
                endForward = fuel.get(fuel.size() - fuelToTake).forward - EXIT_FOLLOW_THROUGH_METERS;
                for (int i = fuel.size() - fuelToTake; i < fuel.size(); i++) {
                    collectedFuel.add(fuel.get(i).position);
                }
            }

            Pose2d entryPose = new Pose2d(pointFromProjection(startForward, lateral, sweepHeading), laneHeading);
            Pose2d exitPose = new Pose2d(pointFromProjection(endForward, lateral, sweepHeading), laneHeading);
            return new LaneTraversal(entryPose, exitPose, collectedFuel);
        }
    }

    private static class LaneTraversal {
        private final Pose2d entryPose;
        private final Pose2d exitPose;
        private final List<Translation2d> collectedFuel;

        private LaneTraversal(Pose2d entryPose, Pose2d exitPose, List<Translation2d> collectedFuel) {
            this.entryPose = entryPose;
            this.exitPose = exitPose;
            this.collectedFuel = collectedFuel;
        }
    }

    private static class ProjectedFuel {
        private final Translation2d position;
        private final double forward;
        private final double lateral;

        private ProjectedFuel(Translation2d position, double forward, double lateral) {
            this.position = position;
            this.forward = forward;
            this.lateral = lateral;
        }
    }
}
