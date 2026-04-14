package frc.robot.subsystems.kicker;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Kicker extends FlywheelMotorSubsystem<MotorIOTalonFX> {

    private enum GainProfile {
        SHOOTING,
        NOT_SHOOTING
    }

    private GainProfile gainProfile = GainProfile.NOT_SHOOTING;

    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(KickerConstants.kFeedForwardVoltage);
    public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(KickerConstants.kFeedBackwardVoltage);

    public static final Setpoint VELOCITY_FORWARD = Setpoint.withVelocitySetpoint(KickerConstants.kFeedForwardVelocity);
    public static final Setpoint VELOCITY_BACKWARD = Setpoint.withVelocitySetpoint(KickerConstants.kFeedBackwardVelocity);

    public static final Setpoint JUGGLE = Setpoint.withVoltageSetpoint(KickerConstants.kJuggleVoltage);
    public Kicker() {
        super(
            KickerConstants.getMotorIO(),
            "Kicker",
            KickerConstants.kEpsilonThreshold);
    }

    public void setShootingGains(boolean isShooting) {
        GainProfile nextProfile = isShooting ? GainProfile.SHOOTING : GainProfile.NOT_SHOOTING;
        if (nextProfile == gainProfile) {
            return;
        }

        KickerConstants.VelocityGains gains = isShooting
                ? KickerConstants.kShootingVelocityGains
                : KickerConstants.kNotShootingVelocityGains;

        io.changeMainConfig(config -> {
            KickerConstants.applyVelocityGains(config, gains);
            return config;
        });
        io.changeFollowerConfig(config -> {
            KickerConstants.applyVelocityGains(config, gains);
            return config;
        });

        gainProfile = nextProfile;
    }
}
