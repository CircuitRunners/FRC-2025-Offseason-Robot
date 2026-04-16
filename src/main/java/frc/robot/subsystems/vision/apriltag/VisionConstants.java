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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldLayout;

import java.util.ArrayList;
import java.util.List;

public class VisionConstants {

  public static AprilTagFieldLayout aprilTagLayout =
      FieldLayout.kAprilTagMap;

  public static String camera0Name = "Thrifty_Cam";
  public static String camera1Name = "Back_Camera";

  // public static Transform3d robotToCamera0 =
  //     new Transform3d(
  //         Units.inchesToMeters(12.564),
  //         Units.inchesToMeters(8.0),
  //         Units.inchesToMeters(7.523),
  //         new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));
  // public static Transform3d robotToCamera1 =
  //     new Transform3d(
  //         Units.inchesToMeters(-11.564),
  //         Units.inchesToMeters(0.0),
  //         Units.inchesToMeters(7.523),
  //         new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(180.0)));

  public static Transform3d robotToCamera0 = new Transform3d(Units.inchesToMeters(-1), Units.inchesToMeters(0), Units.inchesToMeters(21.082),
    new Rotation3d(0.0, Math.toRadians(-28.0), Math.toRadians(0)));
  public static Transform3d robotToCamera1 = new Transform3d(Units.inchesToMeters(-12.194), Units.inchesToMeters(-9.053), Units.inchesToMeters(11.700),
    new Rotation3d(0, Math.toRadians(-20.0), Math.toRadians(180)));


    

  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;
  public static double maxSingleTagDistanceMeters = 3.5;

  public static double linearStdDevBaseline = 0.02;
  public static double angularStdDevBaseline = 0.06;

  public static boolean useVisionRotation = true;
  public static boolean useVisionRotationSingleTag = false;

  public static double[] cameraStdDevFactors =
      new double[] {
        1.0,
        1.0
      };

  public static final List<Integer> singleTagIdsToReject =
      new ArrayList<>() {
        {

          // add(1);
          // add(2);
          // add(3);
          // add(4);
          // add(5);
          // add(12);
          // add(13);
          // add(14);
          // add(15);
          // add(16);
        }
      };

  public static double linearStdDevMegatag2Factor = 0.5;
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY;
}