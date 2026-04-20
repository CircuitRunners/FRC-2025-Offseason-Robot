package frc.robot.controlboard;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.HubShiftUtil;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;


public class ControlBoard {
    private Drive drive;
	private Shooter shooter;
	private Hood hood;
    private IntakeDeploy intakeDeploy;
	private IntakeRollers intakeRollers;
	private Kicker kicker;
	private Conveyor conveyor;
	private Superstructure superstructure;

	
    public ControlBoard(Drive drive, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, Kicker kicker, Conveyor conveyor, Superstructure superstructure) {
        this.drive = drive;
		this.shooter = shooter;
		this.hood = hood;
		this.intakeDeploy = intakeDeploy;
		this.intakeRollers = intakeRollers;
		this.kicker = kicker;
		this.conveyor = conveyor;
        this.superstructure = superstructure;

    }

    private static ControlBoard instance = null;

    public static ControlBoard getInstance(Drive drive, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, Kicker kicker, Conveyor conveyor, Superstructure superstructure) {
        if (instance == null) {
            instance = new ControlBoard(drive, shooter, hood, intakeDeploy, intakeRollers, kicker, conveyor, superstructure);
        }
        return instance;
    }

    private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Trigger overrideTrigger = driver.rightTrigger(0.1);
	//private OverrideBehavior overrideBehavior = OverrideBehavior.CORAL_SCORE_L4;

	private Trigger rightBumper = driver.rightBumper();

	private Trigger hubActiveOrPassing =
        new Trigger(() -> HubShiftUtil.getShiftedShiftInfo().active() || ShotCalculator.getInstance(drive).getParameters().passing());

	// public static enum OverrideBehavior {
	// 	NONE(() -> Commands.none());

	// 	public final Supplier<Command> action;

	// 	private OverrideBehavior(Supplier<Command> overrideAction) {
	// 		action = overrideAction;
	// 	}
	// }

	// public Command setOverrideBehavior(OverrideBehavior behavior) {
	// 	return Commands.runOnce(() -> overrideBehavior = behavior);
	// }

	public void configureBindings(Drive drive, Superstructure superstructure) {
		driver.start()
				.onTrue(Commands.runOnce(
								() -> drive.getDrivetrain().seedFieldCentric(), drive)
						.ignoringDisable(true));

		operator.rightStick()
				.onTrue(Commands.runOnce(
								() -> drive.getDrivetrain().seedFieldCentric(), drive)
						.ignoringDisable(true));
		driverControls();
		debugControls();
	}

	// public OverrideBehavior getOverrideBehavior() {
	// 	return overrideBehavior;
	// }

	public void driverControls() {
		Superstructure s = superstructure;

		Trigger inLaunchingTolerance = 
          new Trigger(
            () ->
                (!s.headingLockToggle || s.atShotGoal() || RobotState.isAutonomous()
          ));

		// INTAKING ###############################################################################

		driver.rightBumper().whileTrue(s.shakeIntake()).onFalse(
		intakeDeploy.setpointCommand(IntakeDeploy.DEPLOY));

		driver.rightTrigger().whileTrue(s.spit()).onFalse(s.setState(Superstructure.State.DEPLOYED));

 		driver.leftBumper().onTrue(s.tuck());

 		driver.leftTrigger(0.1).and(driver.x().negate())
 				.whileTrue(
 						s.runIntakeIfDeployed());
 						//.withName("Deploy and/or Intake"));

		// driver.rightBumper().onTrue(Commands.parallel(conveyor.setpointCommand(Conveyor.FEED_FORWARD),
		// kicker.setpointCommand(Kicker.FEED_FORWARD)))
		// .onFalse(Commands.parallel(kicker.setpointCommand(Setpoint.withNeutralSetpoint()), conveyor.setpointCommand(Setpoint.withNeutralSetpoint())));

		// SHOOTING ##############################################################################

		driver.leftTrigger(0.1).and(driver.x().or(driver.y())).whileTrue(s.shootAndIntake());

 		driver.b()
		.and(() -> s.ignoreHubState || hubActiveOrPassing.getAsBoolean())
		.and(driver.leftTrigger(0.1).negate())
		.and(inLaunchingTolerance).debounce(0.1, DebounceType.kFalling)
		.whileTrue(s.shootWhenReadyPulse());

		driver.y().whileTrue(s.shootWhenReadyPreset(Units.RotationsPerSecond.of(Units.RPM.of(2100).in(Units.RotationsPerSecond)), Units.Degrees.of(23.0)));

		driver.a().whileTrue(s.shootWhenReadyPreset(Units.RotationsPerSecond.of(Units.RPM.of(1625).in(Units.RotationsPerSecond)), Units.Degrees.of(20)));

		driver.leftStick().onTrue(Commands.runOnce(() -> s.shooterIncrement = s.shooterIncrement.minus(Units.RPM.of(12.5))));

		driver.rightStick().onTrue(Commands.runOnce(() -> s.shooterIncrement = s.shooterIncrement.plus(Units.RPM.of(12.5))));

		driver.x()
		.and(() -> s.ignoreHubState || hubActiveOrPassing.getAsBoolean())
		.and(driver.leftTrigger(0.1).negate())
		.and(inLaunchingTolerance).debounce(0.1, DebounceType.kFalling)
		.whileTrue(s.shootWhenReadyRise());

		// TOGGLES ####################################################################################

		driver.povDown().whileTrue(s.driveBrake().withName("Brake"));

		driver.povRight().onTrue((Commands.runOnce(() -> s.headingLockToggle = !s.headingLockToggle)).andThen(
			Commands.sequence(
				rumbleCommand(Units.Seconds.of(0.1)),
				Commands.waitSeconds(0.05),
				rumbleCommand(Units.Seconds.of(0.1)).onlyIf(() -> s.headingLockToggle == false)
			).ignoringDisable(true)
		));

		driver.back().whileTrue(s.turnToHubAuto());
 	}

	// public Command shootingSetOverrideBehavior(Trigger button) {
	// 	return setOverrideBehavior(OverrideBehavior.ALGAE_HOLD)
	// 			.onlyWhile(button)
	// 			.until(overrideTrigger);
	// }


	private void debugControls() {

		operator.leftTrigger().onTrue(intakeDeploy.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(2)))).onFalse(intakeDeploy.setpointCommand(Setpoint.withNeutralSetpoint()));
		operator.rightTrigger().onTrue(intakeDeploy.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-2)))).onFalse(intakeDeploy.setpointCommand(Setpoint.withNeutralSetpoint()));

		operator.back().onTrue(Commands.runOnce(() ->intakeDeploy.setCurrentPosition(IntakeDeployConstants.kDeployPosition)));

		operator.start().onTrue(Commands.runOnce(() -> intakeDeploy.useSoftLimits(false)));

		operator.rightBumper().onTrue(
			Commands.sequence(
			Commands.runOnce(() -> intakeDeploy.useSoftLimits(false)),
			intakeDeploy.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-2)))))
			.onFalse(
				Commands.sequence(
				intakeDeploy.setpointCommand(Setpoint.withNeutralSetpoint()),
				Commands.waitSeconds(1),
				Commands.runOnce(() -> intakeDeploy.setCurrentPosition(IntakeDeployConstants.kDeployPosition))));

		// operator.leftBumper().onTrue(intakeRollers.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(5)))).onFalse(intakeRollers.setpointCommand(Setpoint.withNeutralSetpoint()));
		// operator.rightBumper().onTrue(intakeRollers.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-11)))).onFalse(intakeRollers.setpointCommand(Setpoint.withNeutralSetpoint()));

		// operator.povRight().onTrue(conveyor.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(5)))).onFalse(conveyor.setpointCommand(Setpoint.withNeutralSetpoint()));
		// //operator.povRight().onTrue(conveyor.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(5)))).onFalse(conveyor.setpointCommand(Setpoint.withNeutralSetpoint()));

		// operator.x().onTrue(shooter.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(4)))).onFalse(shooter.setpointCommand(Setpoint.withNeutralSetpoint()));
		// operator.b().onTrue(shooter.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-5)))).onFalse(shooter.setpointCommand(Setpoint.withNeutralSetpoint()));

		// operator.a().onTrue(kicker.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(5)))).onFalse(kicker.setpointCommand(Setpoint.withNeutralSetpoint()));
		// operator.y().onTrue(kicker.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-5)))).onFalse(kicker.setpointCommand(Setpoint.withNeutralSetpoint()));

		// operator.start().onTrue(hood.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(1)))).onFalse(hood.setpointCommand(Setpoint.withNeutralSetpoint()));
		// operator.back().onTrue(hood.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(-1)))).onFalse(hood.setpointCommand(Setpoint.withNeutralSetpoint()));

		// operator.leftStick().onTrue(superstructure.zero());

		// operator.povLeft().onTrue(
		// 	conveyor.setpointCommand(Conveyor.FEED_FORWARD)
		// 	.alongWith(kicker.setpointCommand(Kicker.FEED_FORWARD).alongWith(intakeRollers.Pulse()))
		// ).onFalse(conveyor.setpointCommand(Conveyor.IDLE)
		// 	.alongWith(kicker.setpointCommand(Kicker.IDLE)).alongWith(intakeRollers.setpointCommand(IntakeRollers.IDLE)));
	}

	public Command rumbleCommand(Time duration) {
		return Commands.sequence(
						Commands.runOnce(() -> {
							setRumble(true);
						}),
						Commands.waitSeconds(duration.in(Units.Seconds)),
						Commands.runOnce(() -> {
							setRumble(false);
						}))
				.handleInterrupt(() -> {
					setRumble(false);
					;
				});
	}

	public void setRumble(boolean on) {
		ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}

// // 	public void configureSysIDTests() {
// // 		// Run SysId routines when holding back/start and X/Y.
// // 		// Note that each routine should be run exactly once in a single log.
// // 		driver.back()
// // 				.and(driver.y())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdDynamic(Direction.kForward));
// // 		driver.back()
// // 				.and(driver.x())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdDynamic(Direction.kReverse));
// // 		driver.start()
// // 				.and(driver.y())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdQuasistatic(Direction.kForward));
// // 		driver.start()
// // 				.and(driver.x())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdQuasistatic(Direction.kReverse));

// // 		// Reset the field-centric heading on left bumper press
// // 		driver.leftBumper().onTrue(drive.getGeneratedDrive().runOnce(() -> drive
// // 				.getGeneratedDrive()
// // 				.seedFieldCentric()));
// // 	}

// // 	public void configureModulePointing() {
// // 		driver.a().whileTrue(drive.getGeneratedDrive().applyRequest(() -> brake));
// // 		driver.b()
// // 				.whileTrue(drive
// // 						.getGeneratedDrive()
// // 						.applyRequest(() ->
// // 								point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
// // 	}
}