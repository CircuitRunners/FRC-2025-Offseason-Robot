// MIT License

// Copyright (c) 2025-2026 Littleton Robotics

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.robot.shooting;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.RobotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.FieldLayout;
import frc.lib.util.Util;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.Bounds;

public class ShotCalculator {
    private Drive drive;
    private static ShotCalculator instance;
    public ShotCalculator(Drive drive) {
        this.drive = drive;
    }

    private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / 0.02));

    private double lastHoodAngle;
    private Rotation2d lastDriveAngle;

    public static ShotCalculator getInstance(Drive drive) {
        if (instance == null) instance = new ShotCalculator(drive);
        return instance;
    }

    public record ShotParameters(
        boolean isValid,
        Rotation2d heading,
        double driveVelocity,
        double hoodAngle,
        double flywheelSpeed,
        double distance,
        double distanceNoLookahead,
        double timeOfFlight,
        boolean passing) {}

    // Cache parameters
    private ShotParameters latestParameters = null;

    private static final double minDistance;
    private static final double maxDistance;
    private static final double passingMinDistance;
    private static final double passingMaxDistance;
    private static final double phaseDelay;
    // presets
    public static final double hubPresetDistance = 1.2;
    public static final double towerPresetDistance = 2.5;
    public static final double trenchPresetDistance = 3.03;
    public static final double outpostPresetDistance = 4.84;

    public static final Setpoint passingIdleSpeed = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(Units.RPM.of(1000).in(Units.RotationsPerSecond)));
    


    // Passing targets
    private static final Distance hubPassLine =
        FieldLayout.center.getMeasureY();
    private static final Distance xPassTarget = Units.Inches.of(67);
    private static final Distance yPassTarget = Units.Inches.of(65);
    // Boxes of bad
    // Under tower
    private static final Bounds towerBound =
        new Bounds( 
            Units.Inches.of(0.0).in(Units.Meters), 
            Units.Inches.of(46.0).in(Units.Meters), 
            Units.Inches.of(129.0).in(Units.Meters), 
            Units.Inches.of(168.0).in(Units.Meters));

    // Behind the hubs
    private static final Bounds nearHubBound =
        new Bounds(
            FieldLayout.neutralZoneNear.in(Units.Meters),
            FieldLayout.neutralZoneNear.plus(Units.Inches.of(65)).in(Units.Meters),
            FieldLayout.rightBumpStart.in(Units.Meters),
            FieldLayout.leftBumpEnd.in(Units.Meters));
    private static final Bounds farHubBound =
        new Bounds(
            FieldLayout.oppAllianceZone.in(Units.Meters),
            FieldLayout.kFieldLength.in(Units.Meters),
            FieldLayout.rightBumpStart.in(Units.Meters),
            FieldLayout.leftBumpEnd.in(Units.Meters));

    static {
        minDistance = 1.2;
        maxDistance = 5.3;
        passingMinDistance = 5.4;
        passingMaxDistance = 17.16;
        phaseDelay = 0.03;

    }

    public static double getMinTimeOfFlight() {
        return getTimeOfFlightForShot(minDistance);
    }

    public static double getMaxTimeOfFlight() {
        return getTimeOfFlightForShot(maxDistance);
    }

    public ShotParameters getParameters() {
        boolean passing =
            FieldLayout.distanceFromAllianceWall(Units.Meters.of(drive.getPose().getX()), RobotConstants.isRedAlliance)
            .gte(FieldLayout.kAllianceZoneX.plus(Units.Inches.of(14)));
        if (latestParameters != null) {
            return latestParameters;
        }

        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = drive.getPose();
        ChassisSpeeds robotRelativeVelocity = drive.getRobotRelativeChassisSpeeds();
        estimatedPose =
            estimatedPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate target
        Translation2d target =
            FieldLayout.handleAllianceFlip(
                passing ? getPassingTarget() : FieldLayout.blueHubCenter, RobotConstants.isRedAlliance);
        Pose2d launcherPosition = estimatedPose.transformBy(ShooterConstants.robotToShooter);
        double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());
        SmartDashboard.putNumber("Shooter to Target", launcherToTargetDistance);

        // Calculate field relative launcher velocity
        var robotVelocity = drive.getFieldRelativeChassisSpeeds();
        var robotAngle = drive.getRotation();
        ChassisSpeeds launcherVelocity =
            Util.transformVelocity(
                robotVelocity, ShooterConstants.robotToShooter.getTranslation(), robotAngle);

        // Account for imparted velocity by robot (launcher) to offset
        double timeOfFlight =
            passing
                ? getPassingTimeOfFlightForShot(launcherToTargetDistance)
                : getTimeOfFlightForShot(launcherToTargetDistance);
        Pose2d lookaheadPose = launcherPosition;
        double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

        for (int i = 0; i < 8; i++) {
            timeOfFlight =
                passing
                    ? getPassingTimeOfFlightForShot(lookaheadLauncherToTargetDistance)
                    : getTimeOfFlightForShot(lookaheadLauncherToTargetDistance);
            double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
            double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    launcherPosition.getRotation());
            lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Account for launcher being off center
        Pose2d lookaheadRobotPose =
            lookaheadPose.transformBy(ShooterConstants.robotToShooter.inverse());
        Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

        // Calculate remaining parameters
        double hoodAngle =
            passing
                ? Units.Degrees.of(getPassingHoodSetpointForShot(lookaheadLauncherToTargetDistance)).in(Units.Degrees)
                : Units.Degrees.of(getHoodSetpointForShot(lookaheadLauncherToTargetDistance)).in(Units.Degrees);
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle; //NaN might be not possible, but i dont belive it
        double driveVelocity =
            driveAngleFilter.calculate(
                driveAngle.minus(lastDriveAngle).getRadians() / 0.02);
        lastDriveAngle = driveAngle;
        lastDriveAngle = driveAngle;

        // Check if inside a box of bad
        var flippedPose = FieldLayout.handleAllianceFlip(estimatedPose, RobotConstants.isRedAlliance);
        boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
        boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
        boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
        boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

         double shooterVelocity =
        passing
            ? getPassingShooterSetpointForShot(lookaheadLauncherToTargetDistance)
            : getShooterSetpointForShot(lookaheadLauncherToTargetDistance);

        // Constructor parameters
        latestParameters =
            new ShotParameters(
                outsideOfBadBoxes
                    && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                    && lookaheadLauncherToTargetDistance
                        <= (passing ? passingMaxDistance : maxDistance),
                driveAngle,
                driveVelocity,
                hoodAngle,
                shooterVelocity,
                lookaheadLauncherToTargetDistance,
                launcherToTargetDistance,
                timeOfFlight,
                passing);

        // Log calculated values
        // Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
        // Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
        // Logger.recordOutput(
        //     "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

        return latestParameters;
    }

    private static Rotation2d getDriveAngleWithLauncherOffset(
        Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
        Rotation2d hubAngle =
            new Rotation2d(
                Math.asin(
                    MathUtil.clamp(
                        ShooterConstants.robotToShooter.getY()
                            / target.getDistance(robotPose.getTranslation()),
                        -1.0,
                        1.0)));
        Rotation2d driveAngle =
            fieldToHubAngle.plus(hubAngle).plus(ShooterConstants.robotToShooter.getRotation());
        return driveAngle;
    }

    public double getNaiveTOF(double distance) {
        return getTimeOfFlightForShot(distance);
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }

    public Translation2d getPassingTarget() {
        Distance flippedY = FieldLayout.handleAllianceFlip(drive.getPose(), RobotConstants.isRedAlliance).getMeasureY();
        boolean mirror = flippedY.gte(FieldLayout.center.getMeasureY());

        // Fixed passing target
        Translation2d flippedGoalTranslation =
            //FieldLayout.handleAllianceFlip(
                new Translation2d(
                    xPassTarget, mirror ? FieldLayout.kFieldWidth.minus(yPassTarget) : yPassTarget)/*,
                    //RobotConstants.isRedAlliance)*/;

        return flippedGoalTranslation;
    }

    /**
     * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
     *
     * @param robotTranslation The translation of the center of the robot.
     * @return The target pose for the aimed robot.
     */
    public static Pose2d getStationaryAimedPose(Translation2d robotTranslation) {
        // Calculate target
        Translation2d target =
            FieldLayout.handleAllianceFlip(FieldLayout.blueHubCenter, RobotConstants.isRedAlliance);

        return new Pose2d(
            robotTranslation, getDriveAngleWithLauncherOffset(new Pose2d(robotTranslation, new Rotation2d()), target));
    }

        // interpolates distance to target for shooter setpoint along regression
        private static double getShooterSetpointForShot(double range) {
            return RegressionMaps.kFlywheelAutoAimMap.get(range);
        }

        // interpolates distance to target for hood setpoint along regression
        private static double getHoodSetpointForShot(double range) {
            return RegressionMaps.kHoodAutoAimMap.get(range);
        }

        // interpolates distance to target for time of flight along regression
        private static double getTimeOfFlightForShot(double range) {
            return RegressionMaps.kTimeOfFlightMap.get(range);
        }

        private static double getPassingShooterSetpointForShot(double range) {
            return RegressionMaps.kPassingFlywheelAutoAimMap.get(range);
        }

        // interpolates distance to target for hood setpoint along regression
        private static double getPassingHoodSetpointForShot(double range) {
            return RegressionMaps.kPassingHoodAutoAimMap.get(range);
        }

        // interpolates distance to target for time of flight along regression
        private static double getPassingTimeOfFlightForShot(double range) {
            return RegressionMaps.kPassingTimeOfFlightMap.get(range);
        }
}