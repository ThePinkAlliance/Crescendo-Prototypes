// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;

    /** Creates a new ExampleSubsystem. */
    public VisionSubsystem() {
        this.camera = new PhotonCamera("Logitech_BRIO");
    }

    public Optional<PhotonTrackedTarget> getSpecificTarget(int id) {
        List<PhotonTrackedTarget> targets = this.camera.getLatestResult().getTargets();
        PhotonTrackedTarget desired_target = null;

        for (PhotonTrackedTarget current_tag : targets) {
            if (current_tag.getFiducialId() == id) {
                desired_target = current_tag;
            }
        }

        return Optional.ofNullable(desired_target);
    }

    public List<PhotonTrackedTarget> getAllTargets() {
        return this.camera.getLatestResult().getTargets();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
