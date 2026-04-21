// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
public class SOTMTrajectory extends Command {
  private final Timer timer = new Timer();
  private final Trajectory<SwerveSample> trajectory;
  private final Drive drive;
  private final Superstructure superstructure;

  public SOTMTrajectory(
      Trajectory<SwerveSample> trajectory,
      Drive drive,
      Superstructure superstructure) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.superstructure = superstructure;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    SwerveSample sample = trajectory.sampleAt(timer.get(), RobotConstants.isRedAlliance).orElseThrow();
    Pose2d targetPose = new Pose2d(sample.getPose().getTranslation(), superstructure.headingSetpoint);
    drive.getDrivetrain().setControl(
        DriveConstants.getPIDToPoseRequestUpdater(drive, targetPose)
            .apply(DriveConstants.PIDToPoseRequest));
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
