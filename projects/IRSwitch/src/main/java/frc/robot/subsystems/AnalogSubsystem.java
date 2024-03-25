// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AnalogSubsystem extends SubsystemBase {
  /** Creates a new AnalogueSubsystem. */
  AnalogInput analog;
  public AnalogSubsystem(int input) {
    analog = new AnalogInput(input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Analog Value", analog.getValue());
    SmartDashboard.putNumber("Analog Voltage", analog.getVoltage());
  }
}
