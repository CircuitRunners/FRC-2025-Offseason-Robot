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

package frc.lib.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule;


public class FollowNonstopTrajectory extends Command {
	
	private static final double linearkP =
			DriveConstants.getAutoAlignTranslationController().getP();
	private static final double linearkD =
			DriveConstants.getAutoAlignTranslationController().getD();
	private static final double thetakP =
			DriveConstants.getAutoAlignHeadingController().getP();
	private static final double thetakD =
			DriveConstants.getAutoAlignHeadingController().getD();		

	private final Timer timer = new Timer();
	private final Trajectory trajectory;
	private final Supplier<Optional<Double>> omegaOverride;
	private final Drive drive;

	private final PIDController xController;
	private final PIDController yController;
	private final PIDController thetaController;

	public FollowNonstopTrajectory(
			Trajectory trajectory, Supplier<Optional<Double>> omegaOverride, Drive drive) {
		this.drive = drive;
		this.trajectory = trajectory;
		this.omegaOverride = omegaOverride;
		xController = new PIDController(linearkP, 0, linearkD, 0.02);
		yController = new PIDController(linearkP, 0, linearkD, 0.02);
		thetaController = new PIDController(thetakP, 0, thetakD, 0.02);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		addRequirements(drive);
	}

	public FollowNonstopTrajectory(Trajectory trajectory, Drive drive) {
		this(trajectory, () -> Optional.empty(), drive);
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
		Pose2d currentPose = drive.getPose();
		var desiredState = trajectory.sample(timer.get());
        Pose2d pose = desiredState.poseMeters;
        double velocity = desiredState.velocityMetersPerSecond;
        double vx = velocity * pose.getRotation().getCos();
        double vy = velocity * pose.getRotation().getSin();
        double omega =
            desiredState.velocityMetersPerSecond
            * desiredState.curvatureRadPerMeter;
		double xOutput =
				xController.calculate(currentPose.getX(), pose.getX()) + vx;
		double yOutput =
				yController.calculate(currentPose.getY(), pose.getY()) + vy;
		double thetaOutput =
				omegaOverride
						.get()
						.orElseGet(
								() ->
										thetaController.calculate(
														currentPose.getRotation().getRadians(),
														pose.getRotation().getRadians())
												+ omega);

        xOutput = MathUtil.clamp(xOutput, DriveConstants.kMaxSpeed.unaryMinus().in(Units.MetersPerSecond), DriveConstants.kMaxSpeed.in(Units.MetersPerSecond));
        yOutput = MathUtil.clamp(yOutput, DriveConstants.kMaxSpeed.unaryMinus().in(Units.MetersPerSecond), DriveConstants.kMaxSpeed.in(Units.MetersPerSecond));
        thetaOutput = MathUtil.clamp(thetaOutput,
            DriveConstants.kMaxAngularRate.unaryMinus().in(Units.RadiansPerSecond),
            DriveConstants.kMaxAngularRate.in(Units.RadiansPerSecond));

		drive.getDrivetrain().setControl(
			new SwerveRequest.FieldCentric()
				.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
				.withVelocityX(xOutput)
				.withVelocityY(yOutput)
				.withRotationalRate(thetaOutput)
		);


	}

	@Override
	
	public boolean isFinished() {
		return timer.hasElapsed(trajectory.getTotalTimeSeconds());
	}

	@Override
	public void end(boolean interrupted) {
		// drive.brake();
	}
}