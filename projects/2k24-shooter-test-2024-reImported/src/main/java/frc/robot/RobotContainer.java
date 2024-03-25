// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.IntakeAction;
import frc.robot.commands.LoadAction;
import frc.robot.commands.IntakeAction.IntakeActionType;
import frc.robot.commands.LoadAction.LoadActionType;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Shooter m_shooter = new Shooter();
  private Angle m_angle = new Angle();
  private Joystick m_joy = new Joystick(0);
  private Loader m_loader = new Loader();
  private Intake m_intake = new Intake();
  BooleanSupplier load;
  BooleanSupplier launch;
  JoystickButton lBumper;
  JoystickButton rBumper;
  BooleanSupplier intake;
  BooleanSupplier outtake;
  JoystickButton yButton;
  JoystickButton aButton;


  // Replace with CommandPS4Controller or CommandJoystick if needed
 // private final CommandXboxController m_driverController =
   //   new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    intake = () -> m_joy.getRawButton(4);
    outtake = () -> m_joy.getRawButton(1);
    aButton = new JoystickButton(m_joy, 4);
    yButton = new JoystickButton(m_joy, 1);
    aButton.whileTrue(new IntakeAction(intake, m_intake, IntakeActionType.INTAKE,0.75));
    yButton.whileTrue(new IntakeAction(outtake, m_intake, IntakeActionType.OUTAKE, 0.75));

    load = () -> m_joy.getRawButton(6);
    launch = () -> m_joy.getRawButton(5);
    lBumper = new JoystickButton(m_joy, 6);
    rBumper = new JoystickButton(m_joy, 5);
    lBumper.whileTrue(new LoadAction(launch, m_loader, LoadActionType.LOAD, 3000));
    rBumper.whileTrue(new LoadAction(load, m_loader, LoadActionType.LAUNCH, 3000));

  }

  public Joystick getJoystick() {
    return m_joy;
  }
  public Shooter getShooter() {
    return m_shooter;
  }

  public Angle getAngle() {
    return m_angle;
  }

  public Intake getIntake() {
    return m_intake;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }

 
}
