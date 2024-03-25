// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Northstar;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final VisionSubsystem m_subsystem;
    private final double TARGET_HEIGHT_INCHES = 53.88;
    private final double CAMERA_HEIGHT_INCHES = 31.5;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public VisionCommand(VisionSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> tag_optional = this.m_subsystem.getSpecificTarget(4);

        if (tag_optional.isPresent()) {
            var tag = tag_optional.get();
            double distance = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES)
                    / Math.tan(Units.degreesToRadians(25 + tag.getPitch()));

            SmartDashboard.putNumber("Tag_Pitch", tag.getPitch());
            SmartDashboard.putNumber("Tag_Yaw", tag.getYaw());
            SmartDashboard.putNumber("Tag_Skew", tag.getSkew());
            SmartDashboard.putNumber("Tag_Pose_Ambiguity", tag.getPoseAmbiguity());
            SmartDashboard.putNumber("Tag_Distance", distance);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
