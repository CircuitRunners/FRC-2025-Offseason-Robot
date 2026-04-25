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
@Logged
public class Robot extends TimedRobot {
  private final RobotContainer mRobotContainer;
  private Command mAutonomousCommand;
  public static final Stopwatch autoTimer = new Stopwatch();

  public static final BatteryLogger batteryLogger = new BatteryLogger();
  private final BatteryIOInputs batteryInputs = new BatteryIOInputs();

  private long disabledLoopCount = 0;
  public Robot() {
    mRobotContainer = new RobotContainer();
    Epilogue.bind(this);
    DriverStation.silenceJoystickConnectionWarning(true);
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

    batteryLogger.setBatteryVoltage(batteryInputs.batteryVoltage);
    batteryLogger.setRioCurrent(batteryInputs.rioCurrent);

    batteryLogger.periodic();

    // Update RobotContainer dashboard outputs
    mRobotContainer.updateDashboardOutputs();

    // Clear shooting parameters
    var shotCalculator = mRobotContainer.getShotCalculator();
    shotCalculator.clearShootingParameters();
  }

  @Override
  public void robotInit() {
    RobotController.setBrownoutVoltage(Units.Volts.of(6.0));
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
    mAutonomousCommand = mRobotContainer.getAutoModeSelector().getSelectedCommand();


		autoTimer.start();

		if (mAutonomousCommand != null) {
			CommandScheduler.getInstance().schedule(mAutonomousCommand);
		}
  }

  @Override
  public void autonomousPeriodic() {

    }

  @Override
  public void autonomousExit() {
    autoTimer.reset();
  }

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
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
  public void simulationPeriodic() {}

  public static class BatteryIOInputs {
    public double batteryVoltage = 12.5;
    public double rioCurrent = 0.0;
  }
}
