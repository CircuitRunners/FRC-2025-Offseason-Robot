// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.Stopwatch;
import frc.robot.energy.BatteryLogger;
import frc.robot.subsystems.drive.Drive;
import frc.sim.FuelPhysicsSim;

@Logged
public class Robot extends TimedRobot {
  private final RobotContainer mRobotContainer;
  private Command mAutonomousCommand;
  public static final Stopwatch autoTimer = new Stopwatch();

  public static final BatteryLogger batteryLogger = new BatteryLogger();
  private final BatteryIOInputs batteryInputs = new BatteryIOInputs();
  private static final Drive drive = new Drive();
  private final FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");

  private long disabledLoopCount = 0;
  public Robot() {
    mRobotContainer = new RobotContainer();
    Epilogue.bind(this);
    DriverStation.silenceJoystickConnectionWarning(true);
    // batteryLogger.setBatteryVoltage(batteryInputs.batteryVoltage);
    // batteryLogger.setRioCurrent(batteryInputs.rioCurrent);
  }

  @Override
  public void robotPeriodic() {
    try {
			Threads.setCurrentThreadPriority(true, 5);
			CommandScheduler.getInstance().run();
			Threads.setCurrentThreadPriority(false, 0);
		} catch (Exception e) {
			SmartDashboard.putString("Logged Robot/Latest Error", e.getMessage());
		}

    // batteryLogger.periodic();

    // Update RobotContainer dashboard outputs
    mRobotContainer.updateDashboardOutputs();

    // Clear shooting parameters
    var shotCalculator = mRobotContainer.getShotCalculator();
    shotCalculator.clearShootingParameters();
  }

  @Override
  public void robotInit() {
    RobotController.setBrownoutVoltage(Units.Volts.of(5.8));
  }

  @Override
  public void disabledInit() {
    disabledLoopCount = 0;
  }

  @Override
  public void disabledPeriodic() {
    disabledLoopCount++;

    if (DriverStation.getAlliance().isPresent()) {
			RobotConstants.isRedAlliance = DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
		} else {
			RobotConstants.isRedAlliance =
					DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1)
							|| DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red2)
							|| DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red3);
		}

    if (disabledLoopCount % 50 == 0) {
      mRobotContainer.zeroIntakeDisabled();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Threads.setCurrentThreadPriority(true, 5);
    mAutonomousCommand = mRobotContainer.getAutoModeSelector().getSelectedCommand();
		// autoTimer.start();

		if (mAutonomousCommand != null) {
			CommandScheduler.getInstance().schedule(mAutonomousCommand);
		}
  }

  @Override
  public void autonomousPeriodic() {

    }

  @Override
  public void autonomousExit() {
    // autoTimer.reset();
    mRobotContainer.setDriveDefault();
  }

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
    mRobotContainer.setDriveDefault();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    ballSim.enable();
    ballSim.placeFieldBalls();

    ballSim.configureRobot(RobotConstants.robotSimWidth, RobotConstants.robotSimLength, RobotConstants.robotSimBumperHeight,
    () -> drive.getPose(), () -> drive.getRobotRelativeChassisSpeeds());
  }

  @Override
  public void simulationPeriodic() {
    ballSim.tick();
  }

  public static class BatteryIOInputs {
    public double batteryVoltage = 12.6;
    public double rioCurrent = 0.0;
  }
}
