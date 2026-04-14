package frc.robot;

import com.ctre.phoenix6.CANBus;


/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 * dont use ports 0-7, those are for drivebase motors
 */
public enum Ports {
	SHOOTER(8, RobotConstants.superstructureBus),
	SHOOTER_FOLLOWER(9, RobotConstants.superstructureBus),
	HOOD(10, RobotConstants.superstructureBus),
	INTAKE_DEPLOY(11, RobotConstants.superstructureBus),
	INTAKE_ROLLERS(12, RobotConstants.superstructureBus),
	KICKER(13, RobotConstants.superstructureBus),
	CONVEYOR(14, RobotConstants.superstructureBus),
	KICKER_FOLLOWER(15, RobotConstants.superstructureBus),
	INTAKE_ROLLERS_FOLLOWER(16, RobotConstants.superstructureBus),
	CLIMBER(17, RobotConstants.superstructureBus),

	PHYSICAL_BUTTON(20, new CANBus("rio"));

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}