package frc.robot.subsystems.pivot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSparkMax;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Pivot extends ServoMotorSubsystem<MotorIOSparkMax> {
	private Elevator elevator;
	public static final Setpoint CORAL_INTAKE = Setpoint.withMotionMagicSetpoint(PivotConstants.kCoralIntake);
	public static final Setpoint REEF_INTAKE = Setpoint.withMotionMagicSetpoint(PivotConstants.kReefIntake);
	public static final Setpoint REEF_PREP = Setpoint.withMotionMagicSetpoint(PivotConstants.kReefPrep);

	public static final Setpoint L1_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL1Score);
	public static final Setpoint L2_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL2Score);
	public static final Setpoint L3_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL3Score);
	public static final Setpoint L4_SCORE = Setpoint.withMotionMagicSetpoint(PivotConstants.kL4Score);

	public static final Setpoint CORAL_HOLD = Setpoint.withMotionMagicSetpoint(PivotConstants.kCoralHold);

	public static final Setpoint JOG_POSITIVE = Setpoint.withVoltageSetpoint(Units.Volts.of(1.0));
	public static final Setpoint JOG_NEGATIVE = Setpoint.withVoltageSetpoint(Units.Volts.of(-1.0));
	public static final Setpoint HOLD = Setpoint.withNeutralSetpoint();

	public static final Setpoint CORAL_STATION = Setpoint.withMotionMagicSetpoint(PivotConstants.kStationIntake);

	private Debouncer resetDebouncer = new Debouncer(0.1, DebounceType.kRising);
	private boolean lastShouldReset = false;

	public Pivot(Elevator elevator) {
		super(
				PivotConstants.getMotorIO(),
				"Pivot",
				Units.Degrees.of(2.0));
			setCurrentPosition(PivotConstants.kCoralIntake);
			this.elevator = elevator;
		}

	public static final StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Pivot", Pose3d.struct)
			.publish();

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();

		try {
			publisher.set(PivotConstants.kOffsetPose.plus(new Transform3d(
					new Translation3d(
							BaseUnits.DistanceUnit.zero(),
							BaseUnits.DistanceUnit.zero(),
							ElevatorConstants.converter.toDistance(elevator.getPosition())),
					new Rotation3d(BaseUnits.AngleUnit.zero(), getPosition(), BaseUnits.AngleUnit.zero()))));
		} catch (Exception e) {
			// TODO: handle exception
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
	}
}