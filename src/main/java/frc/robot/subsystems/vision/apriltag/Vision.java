// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision.apriltag;

import static frc.robot.subsystems.vision.apriltag.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LogUtil;
import frc.lib.logging.LoggedTracer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.apriltag.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.apriltag.VisionIO.PoseObservationType;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;
  private final Alert tagLastSeenAlert =
      new Alert("No April tag seen for longer than 20 seconds.", AlertType.kWarning);

  private double lastTargetSeenTime = 0;
  private final VisionIO.VisionIOInputs[] inputs;

  List<Pose3d> tagPoses = new ArrayList<>();
  List<Pose3d> robotPoses = new ArrayList<>();
  List<Pose3d> robotPosesAccepted = new ArrayList<>();
  List<Pose3d> robotPosesRejected = new ArrayList<>();

  List<Pose3d> allTagPoses = new ArrayList<>();
  List<Pose3d> allRobotPoses = new ArrayList<>();
  
  List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  final int maxPoses = 5;

  public List<Pose3d> allRobotPosesAccepted = new ArrayList<>();

  private final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);

  private boolean ignoreThrifty = false;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIO.VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIO.VisionIOInputs();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    SmartDashboard.putData("ThriftyCam Ignore/Toggle",
        new InstantCommand(() -> ignoreThrifty = !ignoreThrifty));
  }

  @Override
  public void periodic() {
    //LoggedTracer.record("Vision Loop Time");
    SmartDashboard.putBoolean("ThriftyCam Ignore/State", ignoreThrifty);
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      //Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    allTagPoses.clear();
    allRobotPoses.clear();
    
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      tagPoses.clear();

      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        lastTargetSeenTime = observation.timestamp();

        // Check whether to reject pose
        boolean rejectPose = shouldRejectPoseObservation(observation) || (cameraIndex == 0 && ignoreThrifty);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          allRobotPosesAccepted.addAll(robotPosesAccepted);

          while (allRobotPosesAccepted.size() > 50) {
              allRobotPosesAccepted.remove(0);
          }
          // robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double averageDist = observation.averageTagDistance();
        double stdDevFactor =
            (averageDist * averageDist) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (!useVisionRotation || (observation.tagCount() == 1 && !useVisionRotationSingleTag)) {
          angularStdDev = 1000.0;
        }

        stdDevs.set(0, 0, linearStdDev);
        stdDevs.set(1, 0, linearStdDev);
        stdDevs.set(2, 0, angularStdDev);
        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            stdDevs);
      }

      // Log camera datadata
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
    //       tagPoses.toArray(new Pose3d[tagPoses.size()]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
    //       robotPoses.toArray(new Pose3d[robotPoses.size()]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
    //       robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
    //   Logger.recordOutput(
    //       "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
    //       robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    } //for (camera : listOfCameras)

    //Log summary data
    // LogUtil.recordPose3d(
    //     "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    // LogUtil.recordPose3d(
    //     "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    // LogUtil.recordPose3d(
    //     "Vision/Summary/RobotPosesAccepted",
    //     allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    // LogUtil.recordPose3d(
    //     "Vision/Summary/RobotPosesRejected",
    //     allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    // if (timeSinceLastTargetSeen() < 0.25) { 
    //   //statusLed.setColor(StatusLed.kBlue);
    // } else {
    //   //statusLed.setColor(StatusLed.kBlack);
    // }

    tagLastSeenAlert.set(timeSinceLastTargetSeen() >= 20.0);
  }

  /** All criteria to reject a post returned by the vision system */
  private boolean shouldRejectPoseObservation(PoseObservation observation) {
    // Should have at least one tag
    if (observation.tagCount() <= 0) {
      return true;
    }

    // Single tag results can provide more errors than multi-tag
    if (observation.tagCount() == 1) {
      if (observation.ambiguity() > maxAmbiguity) {
        return true;
      }

      // Results get worse with a single tag at distance
      if (observation.averageTagDistance() > maxSingleTagDistanceMeters) {
        return true;
      }
    }

    // Must have realistic Z coordinate, i.e. reject poses which think the robot is hoving above
    // or below the floor.
    if (Math.abs(observation.pose().getZ()) > maxZError) {
      return true;
    }

    // Must be within the field boundaries
    return observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth();
  }

  public double timeSinceLastTargetSeen() {
    return Timer.getFPGATimestamp() - lastTargetSeenTime;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public boolean isConnected() {
    boolean result = true;
    for (var cameraInput : inputs) {
      result &= cameraInput.connected;
    }
    return result;
  }

  public Pose2d getLatestVisionPose() {
    if (allRobotPosesAccepted.size() == 0) return new Pose2d();
    return allRobotPosesAccepted.get(allRobotPosesAccepted.size()-1).toPose2d();
  }
}
