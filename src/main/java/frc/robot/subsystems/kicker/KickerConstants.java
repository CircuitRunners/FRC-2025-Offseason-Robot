package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class KickerConstants {
    private static final double kGearing = (18.0 / 18.0);

	public record VelocityGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

	public static final VelocityGains kNotShootingVelocityGains = new VelocityGains(
			7.0,
			0.0,
			0.0,
			5.7,
			0.09,
			0.0);

	public static final VelocityGains kShootingVelocityGains = new VelocityGains(
			11.0,
			0.0,
			0.0,
			5.7,
			0.09,
			0.0);

    public static final Voltage kFeedForwardVoltage = Volts.of(9.0);
    public static final Voltage kFeedBackwardVoltage = Volts.of(-2.0);
	public static final Voltage kJuggleVoltage = Volts.of(-2);

	public static final AngularVelocity kFeedForwardVelocity = Units.RotationsPerSecond.of(Units.RPM.of(4200).in(Units.RotationsPerSecond));
	public static final AngularVelocity kFeedBackwardVelocity = Units.RotationsPerSecond.of(Units.RPM.of(-1000).in(Units.RotationsPerSecond));

	public static final AngularVelocity kEpsilonThreshold = Units.RotationsPerSecond.of(2.0);

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

        config.CurrentLimits.StatorCurrentLimitEnable = false;//Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = false;	
		config.CurrentLimits.SupplyCurrentLimit = 100.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -800;
        config.TorqueCurrent.TorqueNeutralDeadband = 0;

        config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Second;
		config.unit = Units.Rotations;
		config.mainID = Ports.KICKER.id;
		config.mainBus = Ports.KICKER.bus;
		config.followerConfig = getFXConfig();
		config.followerIDs = new int[] {Ports.KICKER_FOLLOWER.id};
		config.followerMotorAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		config.followerBuses = new CANBus[] {Ports.KICKER_FOLLOWER.bus};
		return config;
	}

    public static MotorIOTalonFX getMotorIO() {
		//if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		// } else {
		// 	return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		// }
	}

    public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = new DCMotor(
				12, 4.05, 275, 1.4, edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.000000001;

		return simConstants;
	}
}
