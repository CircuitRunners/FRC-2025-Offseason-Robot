package frc.lib.io;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

/**
 * MotorIO implementation for a SPARK MAX / NEO
 */
public class MotorIOSparkMax extends MotorIO {
    private final SparkMax main;
    private final SparkMax[] followers;
    private final SparkClosedLoopController closedLoopController;

    public MotorIOSparkMax(MotorIOSparkMaxConfig config) {
        super(config.unit, config.time, config.followerIDs.length);

        main = new SparkMax(config.mainID, config.motorType);

        // base config for the leader
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.inverted(config.inverted);
        sparkConfig.idleMode(config.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        sparkConfig.encoder.positionConversionFactor(config.positionConversionFactor);
        sparkConfig.encoder.velocityConversionFactor(config.velocityConversionFactor);
        sparkConfig.closedLoop.pid(config.kP, config.kI, config.kD);

        main.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopController = main.getClosedLoopController();

        // configure followers
        followers = new SparkMax[config.followerIDs.length];
        for (int i = 0; i < config.followerIDs.length; i++) {
            followers[i] = new SparkMax(config.followerIDs[i], config.motorType);

            SparkMaxConfig followerCfg = new SparkMaxConfig();
            followerCfg.follow(main, config.followerOpposeMain[i]);
            followerCfg.idleMode(config.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

            followers[i].configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
    public void updateInputs() {
        inputs.position = Units.Rotations.of(main.getEncoder().getPosition());
        double rpm = main.getEncoder().getVelocity();
        inputs.velocity = Units.RotationsPerSecond.of(rpm / 60.0);
        inputs.statorCurrent = Units.Amps.of(main.getOutputCurrent());
        inputs.supplyCurrent = Units.Amps.of(0); // not available on SparkMax
        inputs.motorVoltage = Units.Volts.of(main.getBusVoltage() * main.getAppliedOutput());
        inputs.motorTemperature = Units.Celsius.of(main.getMotorTemperature());
    }

    @Override
    protected void setNeutralSetpoint() {
        main.stopMotor();
    }

    @Override
    protected void setCoastSetpoint() {
        main.stopMotor();
    }

    @Override
    protected void setVoltageSetpoint(Voltage voltage) {
        main.setVoltage(voltage.in(Units.Volts));
    }

    @Override
    protected void setDutyCycleSetpoint(Dimensionless percent) {
        main.set(percent.in(Units.Percent) / 100.0);
    }

    @Override
    protected void setMotionMagicSetpoint(Angle mechanismPosition) {
        setPositionSetpoint(mechanismPosition);
    }

    @Override
    protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
        double rps = mechanismVelocity.in(Units.RotationsPerSecond);
        closedLoopController.setReference(rps * 60.0, ControlType.kVelocity);
    }

    @Override
    protected void setPositionSetpoint(Angle mechanismPosition) {
        closedLoopController.setReference(mechanismPosition.in(Units.Rotations), ControlType.kPosition);
    }

    @Override
    public void setCurrentPosition(Angle mechanismPosition) {
        main.getEncoder().setPosition(mechanismPosition.in(Units.Rotations));
    }

    @Override
    public void zeroSensors() {
        setCurrentPosition(Units.Rotations.of(0.0));
    }

    @Override
    public void setNeutralBrake(boolean wantsBrake) {
        IdleMode mode = wantsBrake ? IdleMode.kBrake : IdleMode.kCoast;

        // create concrete SparkMaxConfig then call the mutator (ignore returned
        // SparkBaseConfig)
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(mode); // returns SparkBaseConfig but mutates cfg

        main.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        for (SparkMax f : followers) {
            SparkMaxConfig followerCfg = new SparkMaxConfig();
            followerCfg.idleMode(mode);
            f.configure(followerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void useSoftLimits(boolean enable) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.softLimit.forwardSoftLimitEnabled(enable);
        cfg.softLimit.reverseSoftLimitEnabled(enable);

        main.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        for (SparkMax f : followers) {
            SparkMaxConfig followerCfg = new SparkMaxConfig();
            followerCfg.softLimit.forwardSoftLimitEnabled(enable);
            followerCfg.softLimit.reverseSoftLimitEnabled(enable);
            f.configure(followerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public static class MotorIOSparkMaxConfig {
        public int mainID = -1;
        public int[] followerIDs = new int[0];
        public boolean[] followerOpposeMain = new boolean[0];
        public MotorType motorType = MotorType.kBrushless;
        public boolean inverted = false;
        public boolean brakeMode = true;
        public double positionConversionFactor = 1.0;
        public double velocityConversionFactor = 1.0;
        public double kP = 0.0, kI = 0.0, kD = 0.0;
        public AngleUnit unit = Units.Rotations;
        public TimeUnit time = Units.Seconds;
    }
}
