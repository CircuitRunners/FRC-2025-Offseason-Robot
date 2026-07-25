package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Date;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldLayout;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoConstants;

/**
 * The class represents the drivetrain of the robot. It manages telemetry output, pose tracking, and updating drivetrain  states.
 * I certify the work I am submitting is my original work. I have not shared nor exchanged information from anyone, nor will I do so in the future.
 * Additionally, I will immediately notify a teacher if I become aware that another student has comprimised the policy.
 */
public class Drive extends SubsystemBase {
    /**The most recently read drivetrain state */
    private SwerveDriveState lastReadState;

    /** The swerve drivetrain controller created from tuner constants */
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    /** The maximum speed of the drivetrain in meters per second at 12V */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** Telemetry object for keeping drivetrain data */
    private final Telemetry telemetry = new Telemetry(MaxSpeed);

    /** A WPILib Field2d object used to display the robot pose in the dashboard */
    private final Field2d elasticPose = new Field2d();

    /**
     * Constructs the Drive subsystem.
     * Initializes drivetrain state tracking and registers telemetry.
     */
    public Drive() {
        lastReadState = drivetrain.getState();
        drivetrain.registerTelemetry(telemetry::telemeterize);
        configurePathPlanner();
        SmartDashboard.putData("Drive", this);
        SmartDashboard.putData("Elastic Field 2D", elasticPose);
    }

    /**
     * Returns the {@link CommandSwerveDrivetrain} managed by this subsystem.
     * @return The instance of the drivetrain controller.
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    @Override
    /**
     * Periodic method that updates the drivetrain state and outputs telemetry to dashboard.
     */
    public void periodic() {
        lastReadState = drivetrain.getState();
        outputTelemetry();
    }
    /**
     * Outputs telemetry and pose data to SmartDashboard.
     */
    public void outputTelemetry() {
        // telemetry.telemeterize(lastReadState);
        elasticPose.setRobotPose(getPose());
        // energy draw
        // for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getDrivetrain().getModules()) {
        //     Robot.batteryLogger.reportCurrentUsage("Drivebase", module.getDriveMotor().getSupplyCurrent().getValueAsDouble() + module.getSteerMotor().getSupplyCurrent().getValueAsDouble());
        // }
    }

     /**
     * Returns the current drivetrain state.
     * @return the most recent {@link SwerveDriveState}.
     */
    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    /**
     * Returns the current pose of the robot on the field.
     * @return The robot's pose as a {@link Pose2d}.
     */
    public Pose2d getPose() {
        return lastReadState.Pose;
    }

    public Pose2d getLookaheadPose(Time lookaheadTime) {
        ChassisSpeeds speeds = getRobotRelativeChassisSpeeds();
        Transform2d speedsPose = new Transform2d(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond))
                    .times(lookaheadTime.in(Units.Seconds));
        Pose2d lookaheadPose = getPose().transformBy(speedsPose);
        return lookaheadPose;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public static boolean onOpponentSide(boolean isRedAlliance, Pose2d pose) {
        return (isRedAlliance
                        && pose.getTranslation().getX()
                                < FieldLayout.kFieldLength.in(Units.Meters) / 2 - DriveConstants.kMidlineBuffer)
                || (!isRedAlliance
                        && pose.getTranslation().getX()
                                > FieldLayout.kFieldLength.in(Units.Meters) / 2 + DriveConstants.kMidlineBuffer);
    }
    public boolean onOpponentSide() {
        return onOpponentSide(RobotConstants.isRedAlliance, this.getPose());
    }

    /**
     * Returns the current robot relative chassis speeds
     * @return the most recent {@link ChassisSpeeds}
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * Returns the current field relative chassis speeds
     * @return the most reccent {@link ChassisSpeeds}
     */
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getRotation());
    }

    /**
     * Drives the robot robot centric
     * @param speeds
     */
    public void driveRobotCentric(ChassisSpeeds speeds){
      getDrivetrain().setControl(
           new SwerveRequest.RobotCentric()
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
      .withRotationalRate(speeds.omegaRadiansPerSecond)
    );
    }

    public void followChoreoTrajectory(SwerveSample sample) {
		getDrivetrain().setControl(
				DriveConstants.getPIDToPoseRequestUpdater(this, sample.getPose()).apply(DriveConstants.PIDToPoseRequest));
	}

    /**
     * Resets the drivetrain's odometry to a specific pose.
     * @param pose The new pose to reset odometry to.
     */
    public void resetPose(Pose2d pose) {
        getDrivetrain().resetPose(pose);
    }

    public Command resetPoseCmd(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

    /**
     * Stops all the swerve module motors
     */
    public void stopDrive() {
        for (int i = 0; i < 4; i++ ) {
            drivetrain.getModules()[i].getDriveMotor().stopMotor();
         }
    }

    /**
     * Applies a swerve request that sets the swerve drive module states to point inward on the robot in an "X" fashion, creating a natural brake which will oppose any motion.
     * @return the command that brakes the swerve
     */
    public Command brake() {
        return this.getDrivetrain().applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }
    /**
     * Sets zero velocity
     * @return command
     */
    public Command stopDrivetrain() {
      return Commands.runOnce(() -> getDrivetrain().setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds())));
    }

    private void configurePathPlanner() {
        RobotConfig config = DriveConstants.getRobotConfig();
        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotCentric(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () ->
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                RobotConstants.isRedAlliance,
                this // Reference to this subsystem to set requirements
        );

    }
}