package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.robot.Ports;

public class HoodConstants {
    public static final double kGearing = (25.0 / 12.0) * (181.0 / 10.0);

    public static final Angle kKitbotPosition = Units.Degrees.of(0.0);
    public static final Angle kMaxAngle = Units.Degrees.of(43.3);
    public static final Angle kMinAngle = Units.Degrees.of(20.0); // 8.2 higher

    public static final Angle kEpsilonThreshold = Units.Degrees.of(3.0);
	
	public static final Time retractTime = Units.Seconds.of(0.1);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 110.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.33;
        config.Slot0.kG = 0.3;

		config.Slot2.kP = 1000.0;
		config.Slot2.kD = 110.0;
		config.Slot2.kG = 7.5;
		config.Slot2.kS = 3.0;

		config.Slot2.GravityType = GravityTypeValue.Elevator_Static;
		config.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

		config.MotionMagic.MotionMagicCruiseVelocity = 2.0;
		config.MotionMagic.MotionMagicAcceleration = 15.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxAngle.in(Units.Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinAngle.in(Units.Rotations);

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.HOOD.id;
		config.mainBus = Ports.HOOD.bus;
		config.time = Units.Seconds;
		config.unit = Units.Rotations;
		return config;
	}

    public static MotorIOTalonFX getMotorIO() {
		return new MotorIOTalonFX(getIOConfig());
	}

    public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kMinAngle;
		config.kHomingTimeout = Units.Seconds.of(0.2);
		config.kHomingVoltage = Units.Volts.of(-1.0);
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0);

		return config;
	}
}
