// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleAction extends Command {

  public enum AngleActionType{
    MAX,
    MIN,
    ANGLE,
    IDLE
  }

  private Angle m_angle;
  private double m_position;

  /** Creates a new AngleAction. */
  public AngleAction(Angle angle, AngleActionType type, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle;
    m_position = position;
    addRequirements(m_angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
