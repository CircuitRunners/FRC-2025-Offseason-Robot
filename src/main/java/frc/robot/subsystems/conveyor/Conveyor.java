package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.ContinuousConditionalCommand;

public class Conveyor extends MotorSubsystem<MotorIOTalonFX>{

    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedForwardVoltage);
    public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedBackwardVoltage);
    public static final Setpoint JUGGLE = Setpoint.withVoltageSetpoint(ConveyorConstants.kJuggleVoltage);

    private boolean pulseIn = false;
    private final Timer pulseTimer = new Timer();
    private final Trigger lowPulseCurrentDebouncedTrigger =
            new Trigger(this::isPulseStatorCurrentLowRaw)
                    .debounce(ConveyorConstants.kCurrentDebounceSeconds, DebounceType.kRising);
    private final Trigger lowRiseCurrentDebouncedTrigger =
            new Trigger(this::isRiseStatorCurrentLowRaw)
                    .debounce(ConveyorConstants.kCurrentDebounceSeconds, DebounceType.kRising);
    public boolean isPulsing = false;
    
    public Conveyor() {
        super(ConveyorConstants.getMotorIO(), "Conveyor");
    }

    public boolean isStatorCurrentLowForPulse() {
        return lowPulseCurrentDebouncedTrigger.getAsBoolean();
    }

    public boolean isStatorCurrentLowForRise() {
        return lowRiseCurrentDebouncedTrigger.getAsBoolean();
    }

    private boolean isPulseStatorCurrentLowRaw() {
        return getStatorCurrent().lte(ConveyorConstants.kPulseLowCurrentThreshold) || getStatorCurrent().gte(ConveyorConstants.kPulseHighCurrentThreshold);
    }

    private boolean isRiseStatorCurrentLowRaw() {
        return getStatorCurrent().lte(ConveyorConstants.kRiseCurrentThreshold);
    }

    private void startPulse(boolean in) {
        pulseIn = in;
        pulseTimer.restart();
        this.applySetpoint(Setpoint.withVoltageSetpoint(
                pulseIn ? ConveyorConstants.kPulseInVoltage : ConveyorConstants.kPulseOutVoltage));
        isPulsing = true;
    }

    public Command Pulse() {
        return Commands.startEnd(() -> startPulse(true), () -> {
            this.applySetpoint(IDLE);
            isPulsing = false;
            pulseTimer.stop();}
            , this);
    }

    public Command feedForwardOrPulseOnLowCurrent() {
        return new ContinuousConditionalCommand(
            Pulse(),
            followSetpointCommand(() -> FEED_FORWARD),
            this::isStatorCurrentLowForPulse);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (isPulsing) {
            if (pulseTimer.hasElapsed(pulseIn ? ConveyorConstants.pulseInTime : ConveyorConstants.pulseOutTime)) {
                startPulse(!pulseIn);
            }
        }
    }
}
