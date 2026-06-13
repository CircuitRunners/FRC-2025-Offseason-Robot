package frc.lib.bases;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.Util;

/**
 * Base subsystem for any subsystem that uses motors and requires precise velocity control.
 */
public class FlywheelMotorSubsystem<IO extends MotorIO> extends MotorSubsystem<IO> {
	protected final AngularVelocity epsilonThreshold;
	protected final Debouncer debouncer;

	/**
	 * Creates a FlywheelMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for velocity.
	 */
	public FlywheelMotorSubsystem(
			IO io, String name, AngularVelocity epsilonThreshold, Time debounceTime) {
		super(io, name);
		this.epsilonThreshold = epsilonThreshold;
		debouncer = new Debouncer(debounceTime.in(Seconds));
	}

	public void periodic() {
		super.periodic();
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of a provided velocity.
	 *
	 * @param velocity Velocity to check proximity to.
	 * @return Whether the subsystem is acceptably near the given velocity.
	 */
	public boolean nearVelocity(AngularVelocity velocity) {
		return Util.epsilonEquals(
				velocity.baseUnitMagnitude(), getVelocity().baseUnitMagnitude(), epsilonThreshold.baseUnitMagnitude());
	}

	public boolean nearVelocity(AngularVelocity velocity, AngularVelocity epsilonToApply) {
		return Util.epsilonEquals(
				velocity.baseUnitMagnitude(), getVelocity().baseUnitMagnitude(), epsilonToApply.baseUnitMagnitude());
	}

	public boolean nearSetpointVelocity(Setpoint setpoint) {
		return nearVelocity(AngularVelocity.ofBaseUnits(setpoint.baseUnits, RPM));
	}

	public boolean nearSetpointVelocity(Setpoint setpoint, AngularVelocity epsilon) {
		return nearVelocity(AngularVelocity.ofBaseUnits(setpoint.baseUnits, RPM), epsilon);
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of it's velocity setpoint.
	 *
	 * @return Whether the subsystem is acceptably near it's setpoint's velocity. Returns false if not in velcity coontrol mode.
	 */
	public boolean spunUp() {
		return nearVelocity(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(io.getSetpoint().baseUnits))
				&& io.getSetpoint().mode.isVelocityControl();
	}

	public boolean spunUp(AngularVelocity epsilon) {
		return nearVelocity(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(io.getSetpoint().baseUnits), epsilon)
				&& io.getSetpoint().mode.isVelocityControl();
	}

	public boolean spunUpDebounced() {
		return debouncer.calculate(spunUp());
	}

	public boolean spunUpDebounced(AngularVelocity epsilon) {
		return debouncer.calculate(spunUp(epsilon));
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return io.getSetpointDoubleInUnits();
	}

	@Override
	public void applySetpoint(Setpoint setpoint) {
		super.applySetpoint(setpoint);
	}

		/**
	 * Enables or disables soft limits.
	 *
	 * @param enable True to enable, soft to disable.
	 */
	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Sets neutral mode to brake or coast.
	 *
	 * @param wantsBrake True for brake, false for coast.
	 */
	public void setNeutralBrake(boolean wantsBrake) {
		io.setNeutralBrake(wantsBrake);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		io.initSendable(builder);
	}

	/**
	 * Creates a Command that waits until the mechanism is near a given velocity.
	 *
	 * @param mechanismVelocity Velocity to evaluate proximity to.
	 * @return A wait command.
	 */
	public Command waitForVelocityCommand(AngularVelocity flywheelVelocity) {
		return Commands.waitUntil(() -> {
			return nearVelocity(flywheelVelocity);
		});
	}

	public Command waitForVelocityCommand(AngularVelocity flywheelVelocity, AngularVelocity epsilon) {
		return Commands.waitUntil(() -> {
			return nearVelocity(flywheelVelocity, epsilon);
		});
	}

	/**
	 * Creates a Command that goes to a setpoint and then waits until the mechanism is the setpoint's velocity.
	 *
	 * @param mechanismVelocity Velocity to evaluate proximity to.
	 * @return A new Command to apply setpoint and wait.
	 */
	public Command setpointCommandWithWait(Setpoint setpoint) {
		return waitForVelocityCommand(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(setpoint.baseUnits))
				.deadlineFor(setpointCommand(setpoint));
	}

}