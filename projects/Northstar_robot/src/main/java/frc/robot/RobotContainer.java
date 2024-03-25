// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.VisionCommandN;
import frc.robot.subsystems.AprilTagSystem;
import frc.robot.subsystems.AprilTagVisionIONorthstar;
import frc.robot.subsystems.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.junction.Logger;

import java.util.List;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    // private final Northstar m_northstar = new Northstar();
    private final AprilTagSystem m_AprilTagSystem = new AprilTagSystem(new AprilTagVisionIONorthstar("northstar_0"));
    private final DifferentialDriveOdometry odo = new DifferentialDriveOdometry(new Rotation2d(), 0.25, 0.25);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_AprilTagSystem.setDataInterfaces(this::visionConsumer, () -> odo.getPoseMeters());
    }

    private void visionConsumer(List<TimestampedVisionUpdate> updates) {
        for (TimestampedVisionUpdate update : updates) {
            Logger.recordOutput("YEAAAAA", update.pose());
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new VisionCommandN(m_AprilTagSystem);
    }
}
