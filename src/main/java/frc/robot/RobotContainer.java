    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot;

    import static edu.wpi.first.units.Units.*;

    import java.util.List;

    import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
    import com.ctre.phoenix6.swerve.SwerveRequest;

    import edu.wpi.first.epilogue.Logged;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.math.trajectory.Trajectory;
    import edu.wpi.first.math.trajectory.TrajectoryConfig;
    import edu.wpi.first.math.trajectory.TrajectoryGenerator;
    import edu.wpi.first.units.Units;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
    import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
    import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
    import frc.lib.drive.DriveMaintainingHeading;
    import frc.lib.drive.DriveToPose;
    import frc.lib.drive.FollowSyncedPIDToPose;
    import frc.lib.drive.FollowTrajectoryCommand;
    import frc.lib.drive.PIDToPoseCommand;
    import frc.lib.util.FieldLayout.Level;
    import frc.lib.drive.DriveMaintainingHeading.DriveHeadingState;
    //import frc.robot.controlboard.ControlBoard;
    import frc.robot.subsystems.drive.Drive;
    import frc.robot.subsystems.drive.DriveConstants;
    import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.superstructure.Superstructure;

    @Logged
    public class RobotContainer {
        private final Drive drive = new Drive();
        private final Elevator elevator = new Elevator();
        private final EndEffector endEffector = new EndEffector();
        private final Pivot pivot = new Pivot(elevator);
        private final Superstructure superstructure = new Superstructure(drive, elevator, endEffector, pivot);

        //private final ControlBoard controlBoard = ControlBoard.getInstance();

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        private final CommandXboxController joystick = new CommandXboxController(0);
        
        private final SendableChooser<DriveHeadingState> headingStateChooser = new SendableChooser<>();
        private DriveHeadingState headingState = DriveHeadingState.NO_HEADING;

        public RobotContainer() {
            //controlBoard.configureBindings(drive, superstructure);
            configureHeadingStateChooser();
            configureBindings();
        }
        
        //configures heading state chooser
        private void configureHeadingStateChooser() {
            headingStateChooser.setDefaultOption("Reef Heading", DriveHeadingState.REEF_HEADING);
            headingStateChooser.addOption("Barge Heading", DriveHeadingState.BARGE_HEADING);
            headingStateChooser.addOption("Processor Heading", DriveHeadingState.PROCESSOR_HEADING);
            headingStateChooser.addOption("Manual Heading", DriveHeadingState.NO_HEADING);
            SmartDashboard.putData("Heading State Selector", headingStateChooser);
        }
        private void configureBindings() {
            joystick.x().whileTrue(followTrajectoryTest);
            joystick.a().whileTrue(pidToPoseTest);
            drive.setDefaultCommand(
                driveCommand
            );
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            // drive.getDrivetrain().setDefaultCommand(
            //     // Drivetrain will execute this command periodically
            //     drive.getDrivetrain().applyRequest(() ->
            //     driveRequest.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            //     )
            // );

            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            // final var idle = new SwerveRequest.Idle();
            // RobotModeTriggers.disabled().whileTrue(
            //     drive.getDrivetrain().applyRequest(() -> idle).ignoringDisable(true)
            //);

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            // joystick.back().and(joystick.y()).whileTrue(drive.getDrivetrain().sysIdDynamic(Direction.kForward));
            // joystick.back().and(joystick.x()).whileTrue(drive.getDrivetrain().sysIdDynamic(Direction.kReverse));
            // joystick.start().and(joystick.y()).whileTrue(drive.getDrivetrain().sysIdQuasistatic(Direction.kForward));
            // joystick.start().and(joystick.x()).whileTrue(drive.getDrivetrain().sysIdQuasistatic(Direction.kReverse));

            // reset the field-centric heading on left bumper press
            joystick.start().onTrue(drive.getDrivetrain().runOnce(() -> drive.getDrivetrain().seedFieldCentric()));
        }

        private Trajectory configureTrajectory() {
            TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kDriveMaxSpeed, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
            Pose2d start = new Pose2d(2, 1, new Rotation2d(0));

            List<Translation2d> waypoints = List.of(
                new Translation2d(3.0, 5),
                new Translation2d(5.0, 0.0)
            );

            Pose2d end = new Pose2d(3.0, 0.0, new Rotation2d(180));

            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                waypoints,
                end,
                config
            );

            return trajectory;
        }
        public DriveHeadingState getHeadingState() {
            return headingState;
        }

        public void updateHeadingState() {
            headingState = headingStateChooser.getSelected();
        }

        public Command getAutonomousCommand() {
            return Commands.print("No autonomous command configured");
        }


        private final DriveMaintainingHeading driveCommand = 
            new DriveMaintainingHeading(drive, this, () -> joystick.getLeftY(), () -> joystick.getLeftX(), () -> joystick.getRightX());

        private final Command syncedPIDToPoseTest =
            new FollowSyncedPIDToPose(drive, superstructure, new Pose2d(5.0, 2.8, Rotation2d.fromDegrees(270)), Level.NET);
        
        private final Command pidToPoseTest =
            new PIDToPoseCommand(drive, superstructure, new Pose2d(2.0, 0, Rotation2d.fromDegrees(0)));

        private final Command followTrajectoryTest = 
            new FollowTrajectoryCommand(drive, configureTrajectory());
        

        

        
    }
