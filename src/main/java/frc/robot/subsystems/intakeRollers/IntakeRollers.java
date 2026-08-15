package frc.robot.subsystems.intakeRollers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

/**
 * Subsystem representing the intake rollers of the robot.
 */
public class IntakeRollers extends MotorSubsystem<MotorIOTalonFX> {
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kExhaustVoltage);

    public static final Setpoint SLOWTAKE = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kSlowtakeVoltage);

    public static final Setpoint PULSEIN = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPulseInVoltage);
    public static final Setpoint PULSEOUT = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPulseOutVoltage);


    private boolean pulseIn = true;
    private final Timer pulseTimer = new Timer();
    public boolean isPulsing = false;

    /**
     * Constructs a new {@code IntakeRollers} instance.
     */
    public IntakeRollers() {
        super(IntakeRollerConstants.getMotorIO(), "Intake Rollers");
    }

    /**
     * Starts intake roller pulsing.
     * 
     * @param in pulse outward or inward
     */
    private void startPulse(boolean in) {
        pulseIn = in;
        pulseTimer.restart();
        this.applySetpoint(Setpoint.withVoltageSetpoint(pulseIn ? IntakeRollerConstants.kPulseInVoltage : IntakeRollerConstants.kPulseOutVoltage));
        isPulsing = true;
    }

    /**
     * Returns a command that pulses the intake rollers.
     * 
     * @return A command that pulses the intake rollers
     */
    public Command Pulse() {
        return Commands.startEnd(() -> startPulse(true), () -> {
            this.applySetpoint(IDLE);
            isPulsing = false;
            pulseTimer.stop();}
            , this);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (isPulsing) {
            if (pulseTimer.hasElapsed(pulseIn ? IntakeRollerConstants.pulseInTime : IntakeRollerConstants.pulseOutTime)) {
                startPulse(!pulseIn);
            }
        }
    }
}
