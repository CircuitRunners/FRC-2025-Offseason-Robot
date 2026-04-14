package frc.robot.subsystems.intakeRollers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;


public class IntakeRollers extends MotorSubsystem<MotorIOTalonFX> {
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kExhaustVoltage);

    public static final Setpoint PULSEIN = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPulseInVoltage);
    public static final Setpoint PULSEOUT = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPulseOutVoltage);


    private boolean pulseIn = true;
    private final Timer pulseTimer = new Timer();
    public boolean isPulsing = false;

    public IntakeRollers() {
        super(IntakeRollerConstants.getMotorIO(), "Intake Rollers");
    }

    private void startPulse(boolean in) {
        pulseIn = in;
        pulseTimer.restart();
        this.applySetpoint(Setpoint.withVoltageSetpoint(pulseIn ? IntakeRollerConstants.kPulseInVoltage : IntakeRollerConstants.kPulseOutVoltage));
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
                startPulse(!pulseIn);
            }
        }
    }
}
