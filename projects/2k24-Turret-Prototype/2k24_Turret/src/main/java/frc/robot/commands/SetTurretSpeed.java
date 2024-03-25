// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretSpeed extends Command {

  private final TurretSubsystem m_subsystem;
  private final double m_targetSpeed;

  /** Creates a new setTurretSpeed. */
  public SetTurretSpeed(TurretSubsystem subsystem, int targetSpeed) {
    m_subsystem = subsystem;
    m_targetSpeed = targetSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_controller.axisGreaterThan(3, 0.2).onTrue(setSpeedClockwiseCommand(5000));
    // m_controller.axisLessThan(3, 0.2).onTrue(setSpeedClockwiseCommand(0));
    m_subsystem.setRPM(m_targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
