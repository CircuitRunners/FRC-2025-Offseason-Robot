package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;

public class ShooterConstants {
    public static final double kGearing = 1.0 / 1.0;
    public static Transform2d robotToShooter = new Transform2d(Units.Inches.of(-6.881), Units.Inches.of(0), Rotation2d.kZero);

	public record VelocityGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

	public static final VelocityGains kNotShootingVelocityGains = new VelocityGains(
			10.5,
			0.0,
			0.0,
			4.9,
			0.035,
			0.0);

	public static final VelocityGains kShootingVelocityGains = new VelocityGains(
			12.0, // make 6767 for emergency bang bang
			0.0,
			0.0,
			5.25,
			0.04,
			0.0);

	public static final AngularVelocity kIdleSpinup = Units.RPM.of(1500);

    public static final AngularVelocity kJuggleVelocity = Units.RPM.of(360.0);

    public static final AngularVelocity kEpsilonThreshold = Units.RPM.of(60.0);
	public static final Time VELOCITY_THRESHOLD_DEBOUNCE_TIME = Units.Seconds.of(0.0);

	public static void applyVelocityGains(TalonFXConfiguration config, VelocityGains gains) {
		config.Slot1.kP = gains.kP();
		config.Slot1.kI = gains.kI();
		config.Slot1.kD = gains.kD();
		config.Slot1.kS = gains.kS();
		config.Slot1.kV = gains.kV();
		config.Slot1.kA = gains.kA();
	}

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
		applyVelocityGains(config, kNotShootingVelocityGains);

        config.CurrentLimits.StatorCurrentLimitEnable = false; //Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = false; //Robot.isReal();	
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.3;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -800;
		//config.TorqueCurrent.PeakReverseTorqueCurrent = -2; emergency bang bang
        config.TorqueCurrent.TorqueNeutralDeadband = 0;

        config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minutes;
		config.unit = Units.Rotations;
		config.mainID = Ports.SHOOTER.id;
		config.mainBus = Ports.SHOOTER.bus;
		config.followerConfig = getFXConfig();
		config.followerIDs = new int[] {Ports.SHOOTER_FOLLOWER.id};
		config.followerMotorAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		config.followerBuses = new CANBus[] {Ports.SHOOTER_FOLLOWER.bus};
		return config;
	}

    public static MotorIOTalonFX getMotorIO() {
		//if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		//} else {
			//return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		//}
	}

    public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = DCMotor.getKrakenX60Foc(4);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.000000001;

		return simConstants;
	}
}
