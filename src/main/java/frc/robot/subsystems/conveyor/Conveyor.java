package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.robot.subsystems.intakeRollers.IntakeRollerConstants;
import frc.lib.io.MotorIO.Setpoint;

public class Conveyor extends MotorSubsystem<MotorIOTalonFX>{

    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedForwardVoltage);
    public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedBackwardVoltage);
    public static final Setpoint JUGGLE = Setpoint.withVoltageSetpoint(ConveyorConstants.kJuggleVoltage);

    private boolean pulseIn = true;
    private final Timer pulseTimer = new Timer();
    public boolean isPulsing = false;

    public Trigger canPulse =
        new Trigger(
            () ->
            getStatorCurrent().lte(Units.Amps.of(40))
        );
    
    public Conveyor() {
        super(ConveyorConstants.getMotorIO(), "Conveyor");
    }

    private void startPulse(boolean in) {
        pulseIn = in;
        pulseTimer.restart();
        this.applySetpoint(Setpoint.withVoltageSetpoint(pulseIn ? IntakeRollerConstants.kIntakeVoltage : IntakeRollerConstants.kExhaustVoltage));
        isPulsing = true;
    }

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
                if (canPulse.debounce(0.5, DebounceType.kRising).getAsBoolean()) {
                    startPulse(!pulseIn);
                }
            }
        }
    }
}

