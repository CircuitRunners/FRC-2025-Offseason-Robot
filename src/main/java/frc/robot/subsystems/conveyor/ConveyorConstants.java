package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class ConveyorConstants {
    private static final double kGearing = (36.0 / 12.0);

    public static final Voltage kFeedForwardVoltage = Volts.of(-12.0);
    public static final Voltage kFeedBackwardVoltage = Volts.of(4.0);
	public static final Voltage kJuggleVoltage = Volts.of(-4);
    public static final Voltage kPulseInVoltage = kFeedForwardVoltage;
    public static final Voltage kPulseOutVoltage = kFeedBackwardVoltage;
    public static final Current kPulseCurrentThreshold = Units.Amps.of(30.0);
    public static final Current kWiggleCurrentThreshold = Units.Amps.of(45.0);
    public static final double kCurrentDebounceSeconds = 0.5;

	public static final double pulseOutTime = 0.20;
	public static final double pulseInTime = 0.80;

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = false;//Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 150;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.3;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getFXConfig();
        config.time = Minute;
        config.unit = Rotations; 
        config.mainID = Ports.CONVEYOR.id; // PH
        config.mainBus = Ports.CONVEYOR.bus; // PH
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
