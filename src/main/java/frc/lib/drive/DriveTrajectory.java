// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class DriveTrajectory extends Command {
  private static final double linearkP =
      DriveConstants.getAutoAlignTranslationController().getP();
  private static final double linearkD =
      DriveConstants.getAutoAlignTranslationController().getD();
  private static final double thetakP =
      DriveConstants.getAutoAlignHeadingController().getP();
  private static final double thetakD =
      DriveConstants.getAutoAlignHeadingController().getD();
      

  private final Timer timer = new Timer();
  private final Trajectory<SwerveSample> trajectory;
  private final Supplier<Optional<Double>> omegaOverride;
  private final Drive drive;
  private final BooleanSupplier mirror;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private final SwerveRequest.FieldCentric driveNoHeading = 
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public DriveTrajectory(
      Trajectory<SwerveSample> trajectory,
      Supplier<Optional<Double>> omegaOverride,
      Drive drive,
      BooleanSupplier mirror) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.omegaOverride = omegaOverride;
    this.mirror = mirror;
    xController = new PIDController(linearkP, 0, linearkD, 0.02);
    yController = new PIDController(linearkP, 0, linearkD, 0.02);
    thetaController = new PIDController(thetakP, 0, thetakD, 0.02);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  public DriveTrajectory(
      Trajectory<SwerveSample> trajectory, Supplier<Optional<Double>> omegaOverride, Drive drive) {
    this(trajectory, omegaOverride, drive, () -> false);
  }

  public DriveTrajectory(Trajectory<SwerveSample> trajectory, Drive drive, BooleanSupplier mirror) {
    this(trajectory, () -> Optional.empty(), drive, mirror);
  }

  public DriveTrajectory(Trajectory<SwerveSample> trajectory, Drive drive) {
    this(trajectory, () -> Optional.empty(), drive, () -> false);
  }

  @Override
  public void initialize() {
    timer.restart();
    xController.reset();
    yController.reset();
    thetaController.reset();

    
  }

  @Override
  public void execute() {
    // if (linearkP.hasChanged(hashCode())
    //     || linearkD.hasChanged(hashCode())
    //     || thetakP.hasChanged(hashCode())
    //     || thetakD.hasChanged(hashCode())) {
    //   xController.setPID(linearkP.get(), 0, linearkD.get());
    //   yController.setPID(linearkP.get(), 0, linearkD.get());
    //   thetaController.setPID(thetakP.get(), 0, thetakD.get());
    // }

    Pose2d currentPose = drive.getPose();
    SwerveSample trajectoryState =
        trajectory.sampleAt(timer.get(), true).get();

    SwerveSample desiredState =
        trajectoryState;

    double xOutput =
        xController.calculate(currentPose.getX(), desiredState.getPose().getX()) + desiredState.vx;
    double yOutput =
        yController.calculate(currentPose.getY(), desiredState.getPose().getY()) + desiredState.vy;
    double thetaOutput =
        omegaOverride
            .get()
            .orElseGet(
                () ->
                    thetaController.calculate(
                            currentPose.getRotation().getRadians(),
                            desiredState.getPose().getRotation().getRadians())
                        + desiredState.omega);

    drive.getDrivetrain().setControl(
        driveNoHeading
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withVelocityX(xOutput)
        .withVelocityY(yOutput)
        .withRotationalRate(thetaOutput)

        );    
  }

  @Override
  //@AutoLogOutput(key = "DriveTrajectory/isFinished")
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopDrivetrain();
  }
}