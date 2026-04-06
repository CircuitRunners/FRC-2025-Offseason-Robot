package frc.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.util.TunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class IntakeDeployConstants {
    public static final double kGearing = (52.0 / 10.0) * (36.0 / 12.0);

	public static final Angle kDeployPosition = Units.Degrees.of(0);
	public static final Angle kStowPosition = Units.Degrees.of(124);

    public static final Angle kUpShakePosition = Units.Degrees.of(50);

	public static final Angle kExhaustPosition = kDeployPosition;
	public static final Distance kArmLength = Units.Inches.of(14.0);

	public static final Angle kEpsilonThreshold = Units.Degrees.of(6.0);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 40.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kG = 0.7;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = -0.005;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = 0.6;
        config.MotionMagic.MotionMagicAcceleration = 15.0;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        config.Feedback.SensorToMechanismRatio = kGearing;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kStowPosition.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPosition.in(Units.Rotations);

        return config;

    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getFXConfig();
        config.mainID = Ports.INTAKE_DEPLOY.id;
        config.mainBus = Ports.INTAKE_DEPLOY.bus;
        config.time = Seconds;
        config.unit = Degrees;
        return config;
    }

    public static MotorIOTalonFX getMotorIO() {
        //if (Robot.isReal()) {
            return new MotorIOTalonFX(getIOConfig());
        // } else {
        //     return new MotorIOTalonFXSim(getIOConfig(), new PivotSim(getSimConstants()));
        // }
    }

    public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.armLength = kArmLength;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.6046665376);
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.mechanismMaxHardStop = kStowPosition;
		simConstants.mechanismMinHardStop = kDeployPosition;
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kStowPosition;

		return simConstants;
	}

    public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kDeployPosition;
		config.kHomingTimeout = Seconds.of(0.2); 
		config.kHomingVoltage = Units.Volts.of(-1);
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0);

		return config;
	}
}
