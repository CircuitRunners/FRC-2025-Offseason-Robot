package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorConstants {
	private static final double kCoralRollerGearing = (5.0 / 1.0);

	public static final Voltage kSpitVoltage = Units.Volts.of(-3.0);

	public static final Voltage kCoralHoldVoltage = Units.Volts.of(1.0);
	public static final Voltage kCoralIntakeVoltage = Units.Volts.of(6.0);

	public static final Voltage kCoralOuttakeVoltageL1 = Units.Volts.of(-2.0);
	public static final Voltage kCoralOuttakeVoltageL2 = Units.Volts.of(-3.0);
	public static final Voltage kCoralOuttakeVoltageL3 = Units.Volts.of(-3.0);
	public static final Voltage kCoralOuttakeVoltageL4 = Units.Volts.of(-10.0);

	public static final Voltage kSoftCoralOuttakeVoltage = Units.Volts.of(-1.0);

	public static final Voltage kStationIntakeVoltage = Units.Volts.of(12.0);

	public static final Current kCoralStatorCurrentThreshold = Units.Amps.of(60.0);

	public static final MotorIOSparkMaxConfig getIOConfig() {
		MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
		config.mainID = Ports.END_EFFECTOR_MAIN.id; // CAN ID for main roller motor
		config.followerIDs = new int[] { Ports.END_EFFECTOR_FOLLOWER.id }; // optional follower motor
		config.followerOpposeMain = new boolean[] { true }; // set to true if the follower must spin opposite
		config.motorType = MotorType.kBrushless; // NEO motor type
		config.inverted = false;
		config.brakeMode = true;

		config.positionConversionFactor = 1.0 / kCoralRollerGearing;
		config.velocityConversionFactor = 1.0 / kCoralRollerGearing;
		config.kP = 0.045;
		config.kI = 0.0;
		config.kD = 0.0;

		config.unit = Units.Degrees;
		config.time = Units.Seconds;
		return config;
	}

	public static final RollerSimConstants getSimConstants() {
		RollerSimConstants constants = new RollerSimConstants();
		constants.gearing = kCoralRollerGearing;
		constants.momentOfInertia = 0.000000001;
		constants.motor = DCMotor.getNeo550(1);
		return constants;
	}

	public static final MotorIOSparkMax getMotorIO() {
		return new MotorIOSparkMax(getIOConfig());
	}
}