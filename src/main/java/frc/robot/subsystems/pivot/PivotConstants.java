package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.MotorIOSparkMax;
import frc.lib.io.MotorIOSparkMax.MotorIOSparkMaxConfig;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class PivotConstants {
	public static final double kGearing = (25.0 / 1.0);

	public static final Angle kL1Score = Units.Degrees.of(102.0);
	public static final Angle kL2Score = Units.Degrees.of(100.75);
	public static final Angle kL3Score = Units.Degrees.of(100.75);
	public static final Angle kL4Score = Units.Degrees.of(140.0);

	public static final Angle kStationIntake = Units.Degrees.of(80.0);
	public static final Angle kAutoStart = Units.Degrees.of(90.0);

	public static final Angle kCoralIntake = Robot.isReal() ? Units.Degrees.of(-90.0) : Units.Degrees.of(0.0);
	public static final Angle kReefIntake = Units.Degrees.of(-100.0);
	public static final Angle kReefPrep = Units.Degrees.of(-60.0);


	public static final Angle kEndEffectorIdleAfterScoringAngle = Units.Degrees.of(-45.0);

	public static final Angle kCoralHold = Units.Degrees.of(60.0);

	public static final Angle kCoralImpactAngle = Units.Degrees.of(40.0);

	public static final Distance kOutElevatorOffset = Units.Inches.of(9.5);

	public static final int kRotationsToCheck = 10;

	public static final Pose3d kOffsetPose = new Pose3d(
			new Translation3d(-0.1778, 0.0, 0.47),
			new Rotation3d(BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero(), BaseUnits.AngleUnit.zero()));


    public static final MotorIOSparkMaxConfig getIOConfig() {
		MotorIOSparkMaxConfig config = new MotorIOSparkMaxConfig();
		config.mainID = Ports.PIVOT.id; // CAN ID for main roller motor
		config.followerIDs = new int[] { }; // optional follower motor
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
        config.forwardSoftLimitThreshold = Units.Rotations.of(999.0).in(Units.Rotations);

        config.reverseSoftLimitEnabled = true;
        config.reverseSoftLimitThreshold = Units.Rotations.of(-999.0).in(Units.Rotations);

		config.unit = Units.Degrees;
		config.time = Units.Seconds;
		return config;
	}

	public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.111571209);
		simConstants.motor = DCMotor.getNEO(1);
		simConstants.armLength = kOutElevatorOffset;
		simConstants.mechanismMinHardStop = Units.Rotations.of(-999.0);
		simConstants.mechanismMaxHardStop = Units.Rotations.of(999.0);
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kCoralIntake;
		return simConstants;
	}

	public static final MotorIOSparkMax getMotorIO() {
		return new MotorIOSparkMax(getIOConfig());
	}
}