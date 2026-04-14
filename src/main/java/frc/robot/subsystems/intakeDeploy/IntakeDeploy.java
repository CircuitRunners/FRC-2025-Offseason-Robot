package frc.robot.subsystems.intakeDeploy;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class IntakeDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {
    
    public static final Setpoint IDLE = Setpoint.withCoastSetpoint();
    public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kStowPosition);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kDeployPosition);
	public static final Setpoint EXHAUST = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kExhaustPosition);

    public static final Setpoint SHAKE = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kUpShakePosition);

    public static final Setpoint RISE_UP = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kRiseUpPosition);
    public static final Setpoint FALL_DOWN = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kFallDownPosition);

    public IntakeDeploy() {
        super(
            IntakeDeployConstants.getMotorIO(),
            "Intake Deploy",
            IntakeDeployConstants.kEpsilonThreshold//,
            //IntakeDeployConstants.getServoHomingConfig()
        );
        setCurrentPosition(IntakeDeployConstants.kStowPosition);
        //applySetpoint(STOW);
    }

    public void setMotionMagicConstraints(AngularVelocity velocity, AngularAcceleration acceleration) {
        io.setDynamicMotionMagicConstraints(velocity, acceleration);
    }

    public void resetMotionMagicConstraints() {
        setMotionMagicConstraints(
                IntakeDeployConstants.kDefaultCruiseVelocity,
                IntakeDeployConstants.kDefaultAcceleration);
    }

    public Command setMotionMagicConstraintsCommand(AngularVelocity velocity, AngularAcceleration acceleration) {
        return runOnce(() -> setMotionMagicConstraints(velocity, acceleration));
    }

    public Command resetMotionMagicConstraintsCommand() {
        return runOnce(this::resetMotionMagicConstraints);
    }
}
