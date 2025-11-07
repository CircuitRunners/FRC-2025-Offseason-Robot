// package frc.robot.controlboard;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.lib.util.FieldLayout.Level;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.DriveConstants;
// import frc.robot.subsystems.superstructure.Superstructure;

// public class ControlBoard {
//     private static ControlBoard instance = null;

//     public static ControlBoard getInstance() {
//         if (instance == null) {
//             instance = new ControlBoard();
//         }
//         return instance;
//     }

//     private CommandXboxController driver = ControlBoardConstants.mDriverController;
// 	private CommandXboxController operator = ControlBoardConstants.mOperatorController;

//     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
// 	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

// 	private final Trigger overrideTrigger = driver.rightTrigger(0.1);
// 	//private OverrideBehavior overrideBehavior = OverrideBehavior.CORAL_SCORE_L4;

// 	private Trigger rightBumper = driver.rightBumper();
// 	private Trigger endEffectorTrigger;

// 	// public static enum OverrideBehavior {
// 	// 	NET_SCORE(() -> Superstructure.mInstance.netScore().andThen(Superstructure.mInstance.tuck())),
// 	// 	PROCESSOR_SCORE(
// 	// 			() -> Superstructure.mInstance.processorScore().andThen(Superstructure.mInstance.tuckAfterProcessor())),
// 	// 	ALGAE_HOLD(() -> Superstructure.mInstance.algaeStow()),
// 	// 	CORAL_SCORE_L1(() -> Superstructure.mInstance.softCoralScore()),
// 	// 	CORAL_SCORE_L2(() -> Superstructure.mInstance.coralScore(Level.L2)),
// 	// 	CORAL_SCORE_L3(() -> Superstructure.mInstance.coralScore(Level.L3)),
// 	// 	CORAL_SCORE_L4(() -> Superstructure.mInstance.coralScore(Level.L4)),
// 	// 	TUCK(() -> Superstructure.mInstance.tuck()),
// 	// 	NONE(() -> Commands.none());

// 	// 	public final Supplier<Command> action;

// 	// 	private OverrideBehavior(Supplier<Command> overrideAction) {
// 	// 		action = overrideAction;
// 	// 	}
// 	// }

//     // public Command setOverrideBehavior(OverrideBehavior behavior) {
// 	// 	return Commands.runOnce(() -> overrideBehavior = behavior);
// 	// }

// 	public boolean getCoralMode() {
// 		return true; //!rightBumper.getAsBoolean() && !Superstructure.mInstance.getHasAlgae();
// 	}

//     public void configureBindings(Drive drive, Superstructure superstructure) {
// 		driver.back()
// 				.onTrue(Commands.runOnce(
// 								() -> drive.getDrivetrain().seedFieldCentric(), drive)
// 						.ignoringDisable(true));

// 		//driverControls(superstructure);
// 		debugControls();
// 	}

//     // public void driverControls(Superstructure superstructure) {
// 	// 	Superstructure s = superstructure;

// 	// 	// MISC ###############################################################################

// 	// 	endEffectorTrigger = new Trigger(() -> s.getEndEffectorCoralBreak());
// 	// 	endEffectorTrigger.onTrue(rumbleCommand(Units.Seconds.of(0.2)));

// 	// 	driver.start().onTrue(s.stationIntakeToHold());

// 	// 	driver.a().onTrue(s.spit().onlyWhile(driver.a()));

// 	// 	driver.y().onTrue(s.prepClimb());

// 	// 	driver.b().onTrue(s.stowClimb());

// 	// 	driver.leftBumper().onTrue(s.tuckOrHold());

// 	// 	overrideTrigger.onFalse(Commands.deferredProxy(() -> overrideBehavior.action.get()));

// 	// 	// INTAKING ###############################################################################

// 	// 	driver.leftTrigger(0.1)
// 	// 			.onTrue(Commands.either(
// 	// 							s.coralIntakeToHold()
// 	// 									.asProxy()
// 	// 									.beforeStarting(Commands.runOnce(() -> s.setForceGulp(false))),
// 	// 							Commands.either(
// 	// 									s.coralIntaketoIndexer(),
// 	// 									s.algaeIntakeToHold()
// 	// 											.asProxy()
// 	// 											.beforeStarting(setOverrideBehavior(OverrideBehavior.ALGAE_HOLD)),
// 	// 									() -> s.getHasAlgae()),
// 	// 							() -> getCoralMode())
// 	// 					.withName("Either Coral Intake or Algae Intake"));

// 	// 	// CORAL MODE ###############################################################################

// 	// 	// Top Left Paddle
// 	// 	bindCoralAutoScore(Level.L1, driver.povRight());

// 	// 	// Top Right Paddle
// 	// 	bindCoralAutoScore(Level.L2, driver.povUp());

// 	// 	// Bottom Left Paddle
// 	// 	bindCoralAutoScore(Level.L3, driver.povLeft());

// 	// 	// Bottom Right Paddle
// 	// 	bindCoralAutoScore(Level.L4, driver.povDown());

// 	// 	// ALGAE MODE ###############################################################################

// 	// 	// Top Left Paddle
// 	// 	driver.povRight()
// 	// 			.and(() -> !getCoralMode())
// 	// 			.onTrue(s.processorPrep())
// 	// 			.onTrue(setOverrideBehavior(OverrideBehavior.PROCESSOR_SCORE));

// 	// 	// Bottom Left Paddle
// 	// 	bindAlgaeReefIntake(driver.povLeft());

// 	// 	// Bottom Right Paddle
// 	// 	bindNetAlignAndScore(driver.povDown());
// 	// }

//     public void debugControls() {

//     }
// }
