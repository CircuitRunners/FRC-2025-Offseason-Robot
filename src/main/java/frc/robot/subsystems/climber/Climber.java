package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Climber extends ServoMotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint ZERO = Setpoint.withMotionMagicSetpoint(ClimberConstants.kZeroHeight);
	public static final Setpoint CLIMB = Setpoint.withMotionMagicSetpoint(ClimberConstants.kClimbHeight);

	public Climber() {
		super(
				ClimberConstants.getMotorIO(),
				"Climber",
				ClimberConstants.kEpsilonThreshold,
				ClimberConstants.getServoConfig());
		setCurrentPosition(ClimberConstants.kZeroHeight);
		//applySetpoint(ZERO);
	}

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();
	}
}