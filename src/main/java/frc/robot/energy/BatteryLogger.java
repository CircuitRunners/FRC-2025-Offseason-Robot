// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.energy;

import java.util.HashMap;
import java.util.Map;
import lombok.Setter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Class for logging current, power, and energy usage. */
public class BatteryLogger {
  private double totalCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;
  @Setter private double batteryVoltage = 12.6;
  @Setter private double rioCurrent = 0.0;

  private Map<String, Double> subsytemCurrents = new HashMap<>();
  private Map<String, Double> subsytemPowers = new HashMap<>();
  private Map<String, Double> subsytemEnergies = new HashMap<>();

  public void reportCurrentUsage(String key, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.abs(amp);

    double power = totalAmps * batteryVoltage;
    double energy = power * 0.02;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsytemCurrents.put(key, totalAmps);
    subsytemPowers.put(key, power);
    subsytemEnergies.merge(key, energy, Double::sum);

    String[] keys = key.split("/|-");
    if (keys.length < 2) {
      return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) {
        subkey += "/";
      }
      subsytemCurrents.merge(subkey, totalAmps, Double::sum);
      subsytemPowers.merge(subkey, power, Double::sum);
      subsytemEnergies.merge(subkey, energy, Double::sum);
    }
  }

  public void periodic() {
    reportCurrentUsage("Controls/roboRIO", rioCurrent);
    reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
    reportCurrentUsage("Controls/Pigeon", 0.04);
    reportCurrentUsage("Controls/CANivore", 0.03);
    reportCurrentUsage("Controls/Radio", 0.5);

    // Log total and subsystem energy usage
    SmartDashboard.putNumber("EnergyLogger/Current in Amps", totalCurrent);
    SmartDashboard.putNumber("EnergyLogger/Power in Watts", totalPower);
    SmartDashboard.putNumber("EnergyLogger/Energy in Watt Hours", joulesToWattHours(totalEnergy));

    for (var entry : subsytemCurrents.entrySet()) {
        SmartDashboard.putNumber("EnergyLogger/Current in Amps/" + entry.getKey(), entry.getValue());
      subsytemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemPowers.entrySet()) {
      SmartDashboard.putNumber("EnergyLogger/Power in Watts/" + entry.getKey(), entry.getValue());
      subsytemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsytemEnergies.entrySet()) {
      SmartDashboard.putNumber("EnergyLogger/Energy in Watt Hours/" + entry.getKey(), joulesToWattHours(entry.getValue()));
    }

    // Reset power and curren totals, before next loop
    totalPower = 0.0;
    totalCurrent = 0.0;
  }

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  public double getTotalEnergy() {
    return totalEnergy;
  }

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }
}