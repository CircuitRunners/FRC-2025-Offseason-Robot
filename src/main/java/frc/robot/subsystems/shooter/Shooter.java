package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {

    private enum GainProfile {
        SHOOTING,
        NOT_SHOOTING
    }

    private GainProfile gainProfile = GainProfile.NOT_SHOOTING;

    public static final Setpoint IDLE = Setpoint.withCoastSetpoint();
    public static final Setpoint STOP = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(0));
    public static final Setpoint KITBOT = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(10.0));
    
    public static final Setpoint SPINUP = Setpoint.withVelocitySetpoint(ShooterConstants.kIdleSpinup);

    public static final Setpoint JUGGLE = Setpoint.withVelocitySetpoint(ShooterConstants.kJuggleVelocity);

    public Shooter() {
        super(ShooterConstants.getMotorIO(),
            "Shooter", 
            ShooterConstants.kEpsilonThreshold
        );
    }

    public void setShootingGains(boolean isShooting) {
        GainProfile nextProfile = isShooting ? GainProfile.SHOOTING : GainProfile.NOT_SHOOTING;
        if (nextProfile == gainProfile) {
            return;
        }

        ShooterConstants.VelocityGains gains = isShooting
                ? ShooterConstants.kShootingVelocityGains
                : ShooterConstants.kNotShootingVelocityGains;

        io.changeMainConfig(config -> {
            ShooterConstants.applyVelocityGains(config, gains);
            return config;
        });
        io.changeFollowerConfig(config -> {
            ShooterConstants.applyVelocityGains(config, gains);
            return config;
        });

        gainProfile = nextProfile;
    }
}
