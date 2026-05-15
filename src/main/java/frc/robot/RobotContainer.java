package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.logging.LogUtil;
import frc.lib.util.ContinuousConditionalCommand;
import frc.lib.util.HubShiftUtil;
import frc.lib.drive.DriveMaintainingHeading;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.apriltag.Vision;
import frc.robot.subsystems.vision.apriltag.VisionConstants;
import frc.robot.subsystems.vision.apriltag.VisionIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.VisionIOPhotonVisionSim;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoModeSelector;
@Logged
public class RobotContainer {
    private final Drive drive = new Drive();
    private final Hood hood = new Hood();
    private final Vision vision = new Vision(
        drive.getDrivetrain().getVisionConsumer(),
        (RobotBase.isSimulation())
        ? new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose)
        : new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        (RobotBase.isSimulation())
        ? new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose)
        : new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose)
    );

    private final Shooter shooter = new Shooter();
    private final IntakeDeploy intakeDeploy = new IntakeDeploy();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final Kicker kicker = new Kicker();
    private final Conveyor conveyor = new Conveyor();
    private final Superstructure superstructure = new Superstructure(drive, vision, shooter, hood, intakeDeploy, intakeRollers, kicker, conveyor);
    

    private final ControlBoard controlBoard = ControlBoard.getInstance(drive, shooter, hood, intakeDeploy, intakeRollers, kicker, conveyor, superstructure);
    private final ShotCalculator shotCalculator = ShotCalculator.getInstance(drive);

    private Optional<Boolean> autoWinOverride = Optional.empty();
    private boolean disableAutoSpinup = false;
    private final Orchestra superstructureOrchestra = new Orchestra();
    private String orchestraLastStatus = "Not configured";
    private static final String orchestraSongDashboardKey = "Orchestra/Song File";
    private static final String defaultOrchestraSongFile = "song.chrp";
    // private final Trigger lostAutoOverride = 
    // private final Trigger wonAutoOverride = 

    public ShotCalculator getShotCalculator() {
        return shotCalculator;
    }

    public static SendableChooser<Boolean> autoDelay = new SendableChooser<>();
    public static SendableChooser<Double> autoShootAllFuelTime = new SendableChooser<>();
    private AutoModeSelector mAutoModeSelector;
    private static String mPreviousAutoName;
    public AutoModeSelector getAutoModeSelector() {
        return mAutoModeSelector;
    }
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public RobotContainer() {
        controlBoard.configureBindings(drive, superstructure);
        configureBindings();
        RobotConstants.mAutoFactory = new AutoFactory(
				drive::getPose,
				drive.getDrivetrain()::resetPose,
				drive::followChoreoTrajectory,
				true,
				drive);

        CommandScheduler.getInstance().schedule(RobotConstants.mAutoFactory.warmupCmd());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        mAutoModeSelector = new AutoModeSelector(drive, superstructure, RobotConstants.mAutoFactory);
		mPreviousAutoName = mAutoModeSelector.getSelectedCommand().getName();
        SmartDashboard.putData("Auto Chooser", mAutoModeSelector.getAutoChooser()); 
        AutoHelpers.publishDashboardControls();

        SmartDashboard.putData("Auto Overrides/Force Win",
            new InstantCommand(() -> autoWinOverride = Optional.of(true)));

        SmartDashboard.putData("Auto Overrides/Force Loss",
            new InstantCommand(() -> autoWinOverride = Optional.of(false)));

        SmartDashboard.putData("Auto Overrides/Clear",
            new InstantCommand(() -> autoWinOverride = Optional.empty()));

        SmartDashboard.putData("Hub State/Ignore",
        new InstantCommand(() -> superstructure.ignoreHubState = !superstructure.ignoreHubState));

        SmartDashboard.putData("Shooter Idle/Toggle",
        new InstantCommand(() -> disableAutoSpinup = !disableAutoSpinup));

        SmartDashboard.putData("Vision/Set Current Pose To Vision", resetToVisionPose());

        HubShiftUtil.setAllianceWinOverride(() -> autoWinOverride);
        autoDelay.setDefaultOption("NO DELAY", false);
        autoDelay.addOption("YES DELAY", true);

        SmartDashboard.putData("Auto Delay", autoDelay);

        autoShootAllFuelTime.setDefaultOption("5.0s", 5.0);
        autoShootAllFuelTime.addOption("4.0s", 4.0);
        autoShootAllFuelTime.addOption("3.0s", 3.0);

        SmartDashboard.putData("Auto Shoot All Fuel Time", autoShootAllFuelTime);
        configureSuperstructureOrchestra();

        
        // HubShiftUtil.setAllianceWinOverride(
        // () -> {
        //   if (lostAutoOverride.getAsBoolean()) {
        //     return Optional.of(false);
        //   }
        //   if (wonAutoOverride.getAsBoolean()) {
        //     return Optional.of(true);
        //   }
        //   return Optional.empty();
        // });

        // pretty sure we don't need this, or we need to change it a bit cuz heading lock
        // RobotModeTriggers.autonomous()
		// 		.onFalse(Commands.runOnce(() -> drive.getDrivetrain().setControl(new SwerveRequest.ApplyFieldSpeeds()))
		// 				.ignoringDisable(true));
        shooter.setDefaultCommand(
        new ContinuousConditionalCommand(
            shooter.setpointCommand(Shooter.IDLE),
            shooter.followSetpointCommand(
                () -> {
                  var parameters = ShotCalculator.getInstance(drive).getParameters();
                  var shift = HubShiftUtil.getShiftedShiftInfo();
                  if (!parameters.passing()
                      && (shift.active()
                          || shift.remainingTime() < 5.0
                          || superstructure.ignoreHubState)) {
                    return superstructure.shooterSetpoint;
                  } else {
                    return ShotCalculator.passingIdleSpeed;
                  }
                }),
            () -> disableAutoSpinup));
        // shooter.setDefaultCommand(
        //     shooter.followSetpointCommand(() -> {
        //         var parameters = ShotCalculator.getInstance(drive).getParameters();
        //         var shift = HubShiftUtil.getShiftedShiftInfo();

        //         if (!parameters.passing()
        //                 && (shift.active()
        //                 || shift.remainingTime() < 5.0
        //                 || superstructure.ignoreHubState)) {
        //             return superstructure.shooterSetpoint;
        //         } else {
        //             return ShotCalculator.passingIdleSpeed;
        //         }
        //     })
        // );
        //hood.setDefaultCommand(Commands.defer(() -> hood.trackTargetCommand(superstructure.hoodSetpoint), Set.of(hood)));

    //     for (SubsystemBase s : new SubsystemBase[] {
	// 		// intakeDeploy,
	// 		// intakeRollers,
	// 		// conveyor,
	// 		// superstructure,
    //      // kicker,
    //         shooter,
    //         hood
	// 	}) {
	// 		SmartDashboard.putData(s);
	// 	}
    }

    private String getAutoOverrideState() {
        if (autoWinOverride.isEmpty()) {
            return "No Override";
        }
        return autoWinOverride.get() ? "FORCED WIN" : "FORCED LOSS";
    }

    private void configureSuperstructureOrchestra() {
        registerSuperstructureOrchestraInstruments(
                shooter.getIO(),
                hood.getIO(),
                intakeDeploy.getIO(),
                intakeRollers.getIO(),
                kicker.getIO(),
                conveyor.getIO());

        SmartDashboard.putString(orchestraSongDashboardKey, defaultOrchestraSongFile);
        SmartDashboard.putData(
                "Orchestra/Play Song",
                Commands.runOnce(this::loadAndPlayOrchestraFromDashboard).ignoringDisable(true));
        SmartDashboard.putData(
                "Orchestra/Pause",
                Commands.runOnce(this::pauseOrchestra).ignoringDisable(true));
        SmartDashboard.putData(
                "Orchestra/Stop",
                Commands.runOnce(this::stopOrchestra).ignoringDisable(true));
        orchestraLastStatus = "Ready (" + countSuperstructureOrchestraMotors() + " motors registered)";
    }

    private int countSuperstructureOrchestraMotors() {
        return shooter.getIO().getAllMotors().length
                + hood.getIO().getAllMotors().length
                + intakeDeploy.getIO().getAllMotors().length
                + intakeRollers.getIO().getAllMotors().length
                + kicker.getIO().getAllMotors().length
                + conveyor.getIO().getAllMotors().length;
    }

    private void registerSuperstructureOrchestraInstruments(MotorIOTalonFX... motorIOs) {
        for (MotorIOTalonFX motorIO : motorIOs) {
            for (var motor : motorIO.getAllMotors()) {
                var status = superstructureOrchestra.addInstrument(motor);
                if (!status.isOK()) {
                    orchestraLastStatus = "Add instrument issue: " + status;
                }
            }
        }
    }

    private void loadAndPlayOrchestraFromDashboard() {
        String songFile = SmartDashboard.getString(orchestraSongDashboardKey, defaultOrchestraSongFile).trim();
        if (songFile.isEmpty()) {
            orchestraLastStatus = "No song file configured";
            return;
        }

        var loadStatus = superstructureOrchestra.loadMusic(songFile);
        if (!loadStatus.isOK()) {
            orchestraLastStatus = "Load failed: " + loadStatus;
            return;
        }

        var playStatus = superstructureOrchestra.play();
        orchestraLastStatus = playStatus.isOK() ? "Playing " + songFile : "Play failed: " + playStatus;
    }

    private void pauseOrchestra() {
        var pauseStatus = superstructureOrchestra.pause();
        orchestraLastStatus = pauseStatus.isOK() ? "Paused" : "Pause failed: " + pauseStatus;
    }

    private void stopOrchestra() {
        var stopStatus = superstructureOrchestra.stop();
        orchestraLastStatus = stopStatus.isOK() ? "Stopped" : "Stop failed: " + stopStatus;
    }

    private void configureBindings() {
        // drive.setDefaultCommand(
        //     driveCommand
        // );

        
        // drive.getDrivetrain().setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drive.getDrivetrain().applyRequest(() ->
        //     driveRequest.withVelocityX(-ControlBoardConstants.mDriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-ControlBoardConstants.mDriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-ControlBoardConstants.mDriverController.getRightX() * MaxAngularRate).withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate*0.15) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // drive.getDrivetrain().setDefaultCommand(
        //     drive.getDrivetrain().applyRequest(() ->
        //     driveRequest.withVelocityX(-ControlBoardConstants.mOperatorController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-ControlBoardConstants.mOperatorController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-ControlBoardConstants.mOperatorController.getRightX() * MaxAngularRate).withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate*0.15) // Drive counterclockwise with negative X (left)
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
        //ControlBoardConstants.mDriverController.start().onTrue(drive.getDrivetrain().runOnce(() -> drive.getDrivetrain().seedFieldCentric()));
        ControlBoardConstants.mOperatorController.rightStick().onTrue(resetToVisionPose());

        RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
        RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    }

    public void updateDashboardOutputs() {
        // Publish match time
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        // Update from HubShiftUtil
        SmartDashboard.putNumber(
            "Shifts/Remaining Shift Time",
            Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0));
        SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
        SmartDashboard.putString(
            "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
        SmartDashboard.putBoolean(
            "Shifts/Active First?",
            DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

        SmartDashboard.putString("Auto Overrides/Current State", getAutoOverrideState());
        SmartDashboard.putBoolean("Auto Overrides/Override Active", autoWinOverride.isPresent());

        SmartDashboard.putBoolean("Hub State/Current Ignore State", superstructure.ignoreHubState);

        SmartDashboard.putBoolean("Shooter Idle/State", disableAutoSpinup);
        SmartDashboard.putBoolean("Auto Reset Pose In Auto/Enabled", AutoHelpers.shouldResetPoseInAuto());

        Double selectedShootAllFuelTime = autoShootAllFuelTime.getSelected();
        AutoConstants.shootAllFuelTime = selectedShootAllFuelTime != null ? selectedShootAllFuelTime : 3.0;
        SmartDashboard.putNumber("Auto Shoot All Fuel Time/Selected", AutoConstants.shootAllFuelTime);

        SmartDashboard.putBoolean("Vision/Has Accepted Pose", vision.hasAcceptedVisionPose());
        SmartDashboard.putBoolean("Orchestra/Is Playing", superstructureOrchestra.isPlaying());
        SmartDashboard.putNumber("Orchestra/Time Seconds", superstructureOrchestra.getCurrentTime());
        SmartDashboard.putString("Orchestra/Status", orchestraLastStatus);
        LogUtil.recordPose2d("Vision pose", vision.getLatestVisionPose());
    }
    public void zeroIntakeDisabled() {
        if (intakeDeploy.getPosition().gte(IntakeDeployConstants.kStowPosition)) {
            intakeDeploy.setCurrentPosition(IntakeDeployConstants.kStowPosition);
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public final DriveMaintainingHeading driveCommand = 
        new DriveMaintainingHeading(drive, superstructure, () -> ControlBoardConstants.mDriverController.getLeftY(), () -> ControlBoardConstants.mDriverController.getLeftX(), () -> -ControlBoardConstants.mDriverController.getRightX(), () -> superstructure.maintainHeadingEpsilon);
    
    public void setDriveDefault() {
        drive.setDefaultCommand(
            driveCommand
        );
    }

    public Command resetToVisionPose() {
        return Commands.either(
            Commands.runOnce(() -> drive.resetPose(vision.getLatestVisionPose()), drive),
            Commands.none(),
            vision::hasAcceptedVisionPose)
            .ignoringDisable(true);
    }
}
