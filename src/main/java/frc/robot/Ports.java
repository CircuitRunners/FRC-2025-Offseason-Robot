package frc.robot;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 * Do NOT use ports 0 through 7.
 */
public enum Ports {
	END_EFFECTOR_MAIN(24, "canivore1"),
	END_EFFECTOR_FOLLOWER(25, "canivore1"),
	ELEVATOR_MAIN(21, "canivore1"),
	ELEVATOR_FOLLOWER(22, "canivore1"),
	PIVOT(23, "canivore1"),

	END_EFFECTOR_CORAL_BREAMBREAK(2, "canivore1"),

	PHYSICAL_BUTTON(9, "RioDigitalIn");

	public final int id;
	public final String bus;

	private Ports(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}
}