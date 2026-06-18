package frc.robot;

import com.ctre.phoenix6.CANBus;

import choreo.auto.AutoFactory;


public class RobotConstants {

	static {
		RobotConstants.isRedAlliance = false;
	}

	public static boolean isRedAlliance;
	public static AutoFactory mAutoFactory;
	public static CANBus superstructureBus = new CANBus("superstructure");

	//simulation constants
	public static final double robotSimWidth = 0.4191; //meters
	public static final double robotSimLength = 0.6477; //meters
	//placeholder
	public static final double robotSimBumperHeight = 0.0762; //meters
}