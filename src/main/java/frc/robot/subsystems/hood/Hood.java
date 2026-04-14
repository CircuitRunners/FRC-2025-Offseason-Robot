package frc.robot.subsystems.hood;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Hood extends ServoMotorSubsystem<MotorIOTalonFX>{
    public static final Setpoint KITBOT = Setpoint.withMotionMagicSetpoint(HoodConstants.kKitbotPosition);
    public static final Setpoint ZERO = Setpoint.withMotionMagicSetpoint(HoodConstants.kMinAngle);
    
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
