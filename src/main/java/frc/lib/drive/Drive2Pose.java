package frc.lib.drive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class Drive2Pose extends Command {
    private final Drive drive;
    private final Pose2d targetPose;
    private final SynchronousPIDF h = DriveConstants.getAutoAlignHeadingController();
    private final SynchronousPIDF t = DriveConstants.getAutoAlignTranslationController();
    
    private final PIDController xController = new PIDController(t.getP(), t.getI(), t.getD());
    private final PIDController yController = new PIDController(t.getP(), t.getI(), t.getD());
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(h.getP(), h.getI(), h.getD(), new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularRate.in(Units.RadiansPerSecond), 20));
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading = 
        new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.kMaxSpeed.times(DriveConstants.kDriveJoystickDeadband))
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    public Drive2Pose(Drive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d current = drive.getPose();

        double xSpeed = xController.calculate(current.getX(), targetPose.getX());
        double ySpeed = yController.calculate(current.getY(), targetPose.getY());
        double thetaSpeed =
            thetaController.calculate(current.getRotation().getRadians(),
                                      targetPose.getRotation().getRadians());

        // drive.drive(xSpeed, ySpeed, thetaSpeed, true);
        drive.getDrivetrain().setControl(
            driveWithHeading
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withTargetDirection(targetPose.getRotation())
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drive.getPose();

        return Math.abs(current.getX() - targetPose.getX()) < 0.05 &&
               Math.abs(current.getY() - targetPose.getY()) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        // if (interrupted) {
        //     drivetrain.stop();
        // }
        drive.stopDrivetrain();
    }
}