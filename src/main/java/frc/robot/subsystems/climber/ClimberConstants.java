package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.Ports;
import frc.robot.Robot;

public class ClimberConstants {
	public static final double kGearing = (25.0 / 1.0);

    public static final Angle kMaxHeight = Units.Rotations.of(8);
	public static final Angle kClimbHeight = kMaxHeight;
    public static final Angle kZeroHeight = Units.Rotations.of(0);
    public static final Angle kEpsilonThreshold = Units.Rotations.of(0.2);

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = 16.3;
		FXConfig.Slot0.kD = 0.5;
		FXConfig.Slot0.kS = 0.45;
		FXConfig.Slot0.kG = 0.35;

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxHeight.in(Units.Rotations);

		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kZeroHeight.in(Units.Rotations);

		FXConfig.Feedback.SensorToMechanismRatio = kGearing;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = Ports.CLIMBER.id;
		IOConfig.mainBus = Ports.CLIMBER.bus;
		IOConfig.unit = Units.Rotations;
		IOConfig.time = Units.Second;
		return IOConfig;
	}

	// public static LinearSimConstants getSimConstants() {
	// 	LinearSimConstants simConstants = new LinearSimConstants();
	// 	simConstants.motor = DCMotor.getKrakenX60Foc(1);
	// 	simConstants.gearing = kGearing;
	// 	simConstants.carriageMass = Units.Pounds.of(27);
	// 	simConstants.startingHeight = kZeroHeight
	// 	simConstants.minHeight = kZeroHeight;
	// 	simConstants.maxHeight = kMaxHeight;
	// 	simConstants.simGravity = false;
	// 	simConstants.converter = converter;
	// 	return simConstants;
	// }

	public static MotorIOTalonFX getMotorIO() {
		//if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		// } else {
		// 	return new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
		// }
	}

	public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomePosition = kZeroHeight;
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-0.5);
		servoConfig.kSetHomedVelocity = Units.Rotations.of(0.1).per(Units.Second);

		return servoConfig;
	}
}