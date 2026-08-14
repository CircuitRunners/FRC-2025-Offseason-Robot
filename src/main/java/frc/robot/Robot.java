// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

// import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.LogUtil.GcStatsCollector;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.Stopwatch;
import frc.robot.auto.AutoModeSelector;
import frc.robot.energy.BatteryLogger;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.apriltag.VisionConstants;

public class Robot extends TimedRobot {

  private final RobotContainer mRobotContainer;
  private Command mAutonomousCommand;
  public static final Stopwatch autoTimer = new Stopwatch();
  private boolean autoDelayTimerRunning = false;
  private boolean autoDelayTimerPublished = false;

  public static final BatteryLogger batteryLogger = new BatteryLogger();
  private final BatteryIOInputs batteryInputs = new BatteryIOInputs();
  private final GcStatsCollector m_gcStatsCollector = new GcStatsCollector();

  private AutoModeSelector mAutoModeSelector;

  private double loopOverrunWarningTimeout = 0.1;


  private long disabledLoopCount = 0;
  public Robot() {
    mRobotContainer = new RobotContainer();
    // Epilogue.bind(this);
    mAutoModeSelector = new AutoModeSelector(mRobotContainer.drive, mRobotContainer.superstructure, RobotConstants.mAutoFactory);
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    // batteryLogger.setBatteryVoltage(batteryInputs.batteryVoltage);
    // batteryLogger.setRioCurrent(batteryInputs.rioCurrent);
  }

  @Override
  public void robotPeriodic() {
    
		LoggedTracer.reset();

		try {
			Threads.setCurrentThreadPriority(true, 20);

			double commandSchedulerStart = Timer.getTimestamp();
			//Shooting.mInstance.periodic();
			SmartDashboard.putNumber("Current Timestamp Seconds", Timer.getFPGATimestamp());
			CommandScheduler.getInstance().run();
			double commandSchedulerEnd = Timer.getTimestamp();
			LoggedTracer.record("Commands");
			m_gcStatsCollector.update();
			SmartDashboard.putNumber(
					"Logged Robot/Loop Cycle Time Milliseconds",
					(commandSchedulerEnd - commandSchedulerStart) * 1000.0);
			Threads.setCurrentThreadPriority(false, 0);
		} catch (Exception e) {
			SmartDashboard.putString("Error/Last Loop Error/Last Error Message", e.getMessage());
			SmartDashboard.putNumber("Error/Last Loop Error/Last Error Timestamp", Timer.getFPGATimestamp());
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
    SmartDashboard.putData("Auto Chooser", mAutoModeSelector.getAutoChooser()); 
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    VisionConstants.aprilTagLayout.setOrigin(new Pose3d());
    DataLogManager.start();
try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);
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
    autoTimer.resetAndStart();
    autoDelayTimerRunning = true;
    autoDelayTimerPublished = false;
    DataLogManager.log("Auto Started");
    mAutonomousCommand = mAutoModeSelector.getSelectedCommand();

		if (mAutonomousCommand != null) {
			CommandScheduler.getInstance().schedule(mAutonomousCommand);
		}
  }

  @Override
  public void autonomousPeriodic() {
    if (autoDelayTimerRunning
        && !autoDelayTimerPublished
        && moduleTargetsHaveVelocity(mRobotContainer.drive.getState().ModuleTargets)) {
      SmartDashboard.putNumber("Auto Delay Time", autoTimer.getTimeAsDouble());
      autoTimer.reset();
      autoDelayTimerRunning = false;
      autoDelayTimerPublished = true;
    }
  }

  @Override
  public void autonomousExit() {
    autoDelayTimerRunning = false;
    mRobotContainer.setDriveDefault();
  }

  @Override
  public void teleopInit() {
    DataLogManager.log("Teleop Started");
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
  public void simulationPeriodic() {}

  private static boolean moduleTargetsHaveVelocity(SwerveModuleState[] moduleTargets) {
    for (SwerveModuleState moduleTarget : moduleTargets) {
      if (Math.abs(moduleTarget.speedMetersPerSecond) > 0.0) {
        return true;
      }
    }

    return false;
  }

  public static class BatteryIOInputs {
    public double batteryVoltage = 12.6;
    public double rioCurrent = 0.0;
  }
}
