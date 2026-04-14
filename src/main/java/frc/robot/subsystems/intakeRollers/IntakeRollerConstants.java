package frc.robot.subsystems.intakeRollers;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.TunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;

public class IntakeRollerConstants {
    private static final double kGearing = (24.0 / 18.0);

    public static final Voltage kIntakeVoltage = Volts.of(5.0);
    public static final Voltage kExhaustVoltage = Volts.of(-6.0);

    public static final Voltage kPulseInVoltage = Volts.of(6.0);
    public static final Voltage kPulseOutVoltage = Volts.of(-6.0);

    public static final double pulseOutTime = 0.15;
	public static final double pulseInTime = 0.30;

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 100;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.3;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getFXConfig();
        config.time = Minute;
        config.unit = Rotations;
        config.mainID = Ports.INTAKE_ROLLERS.id;
        config.mainBus = Ports.INTAKE_ROLLERS.bus;
        config.followerConfig = getFXConfig();
        config.followerIDs = new int[] {Ports.INTAKE_ROLLERS_FOLLOWER.id};
        config.followerMotorAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
        config.followerBuses = new CANBus[] {Ports.INTAKE_ROLLERS_FOLLOWER.bus};
        return config;
    }

    public static MotorIOTalonFX getMotorIO() {
		// if (Robot.isReal()) {
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

    public static final double intakeWidth = .61;
    public static final int fuelLimit = 55;
    public static int numberOfFuel = 0;
}
