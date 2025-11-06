package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.util.Util;
import frc.robot.Ports;

public class ElevatorConstants {

	public static final double kGearing = (5.0 / 1.0);

	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(
			Units.Inches.of(2.0).plus(Units.Inches.of(0.125)).div(2.0));

	public static final Pose3d stage1Offset = new Pose3d(-0.152, -0.165, -0.910, new Rotation3d());
	public static final Pose3d stage2Offset = new Pose3d(
			-0.0,
			0.0,
			0.0,
			new Rotation3d(BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero(), Units.Degrees.of(90.0)));

	public static final Distance kMaxHeight = converter.toDistance(Units.Degrees.of(3316));

	public static final Distance kAlgaeLiftDistance = Units.Inches.of(2.0);

	public static final Distance kL1ScoringHeight = Units.Inches.of(0.0);
	public static final Distance kL2ScoringHeight = Units.Inches.of(6.3);
	public static final Distance kL3ScoringHeight = kL2ScoringHeight.plus(Units.Inches.of(16.0));
	public static final Distance kL4ScoringHeight = Units.Inches.of(60.25);

	public static final Distance kClearLowPosition = Units.Inches.of(12.0);
	public static final Distance kClearHighPosition = Units.Inches.of(18.608);
	public static final Distance kCoralHoldPosition = Units.Inches.of(13.21);
	public static final Distance kL4PivotClearHeight = kL4ScoringHeight.minus(Units.Inches.of(32.0));
	public static final Distance kEEUnjameHeight = Units.Inches.of(6.0);

	public static final Distance kStationIntake = Units.Inches.of(7.1);

	public static final Distance kStowPosition = Units.Inches.of(0.0);

	public static final Distance kEpsilonThreshold = Units.Inches.of(1.0);

	public static final Distance kElevatorHighThreshold = kCoralHoldPosition.plus(kEpsilonThreshold);

    public static final MotorIOSparkMaxConfig getIOConfig() {
		MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
		config.mainID = Ports.ELEVATOR_MAIN.id; // CAN ID for main roller motor
		config.followerIDs = new int[] { Ports.ELEVATOR_FOLLOWER.id }; // optional follower motor
		config.followerOpposeMain = new boolean[] { false }; // set to true if the follower must spin opposite
		config.motorType = MotorType.kBrushless; // NEO motor type
		config.inverted = false;
		config.brakeMode = true;

		config.positionConversionFactor = 1.0 / kGearing;
		config.velocityConversionFactor = 1.0 / kGearing;
		config.kP = 0.045;
		config.kI = 0.0;
		config.kD = 0.0;

        config.forwardSoftLimitEnabled = true;
        config.forwardSoftLimitThreshold = converter.toAngle(kL4ScoringHeight).in(Units.Rotations);

        config.reverseSoftLimitEnabled = true;
        config.reverseSoftLimitThreshold = converter.toAngle(kStowPosition).minus(Units.Degrees.of(10.0)).in(Units.Rotations);

		config.unit = converter.getDistanceUnitAsAngleUnit(Units.Inches);
		config.time = Units.Seconds;
		return config;
	}

	public static LinearSimConstants getSimConstants() {
		LinearSimConstants simConstants = new LinearSimConstants();
		simConstants.motor = DCMotor.getNEO(1);
		simConstants.gearing = kGearing;
		simConstants.carriageMass = Units.Pounds.of(27);
		simConstants.startingHeight = kStowPosition;
		simConstants.minHeight = kStowPosition;
		simConstants.maxHeight = kL4ScoringHeight;
		simConstants.simGravity = false;
		simConstants.converter = converter;
		return simConstants;
	}

	public static final MotorIOSparkMax getMotorIO() {
		return new MotorIOSparkMax(getIOConfig());
	}

	public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomePosition = converter.toAngle(kStowPosition);
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-0.5);
		servoConfig.kSetHomedVelocity = converter.toAngle(Units.Inches.of(0.1)).per(Units.Second);

		return servoConfig;
	}
}