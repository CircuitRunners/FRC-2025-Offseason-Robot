package frc.robot.subsystems.hood;

import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Hood extends ServoMotorSubsystem<MotorIOTalonFX>{
    public static final Setpoint KITBOT = Setpoint.withPositionSetpoint(HoodConstants.kKitbotPosition);
    public static final Setpoint ZERO = Setpoint.withPositionSetpoint(HoodConstants.kMinAngle);
    
    public Hood() {
        super(
            HoodConstants.getMotorIO(),
            "Hood",
            HoodConstants.kEpsilonThreshold,
            HoodConstants.getServoHomingConfig()
        );
        setCurrentPosition(HoodConstants.kMinAngle);
    }
}
