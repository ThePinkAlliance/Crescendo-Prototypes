// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private DoubleArraySubscriber botpose_blue_subscriber;
    private DifferentialDrivePoseEstimator estimator;

    /** Creates a new Vision. */
    public VisionSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        this.botpose_blue_subscriber = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});

        this.estimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(5), new Rotation2d(), 2,
                2, new Pose2d());
    }

    public Translation3d getTranslation() {
        double[] data = this.botpose_blue_subscriber.get();

        return new Translation3d(data[0], data[1], data[2]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Pose3d tag_pose = Constants.layout.getTagPose(4).get();

        /**
         * NOTE: Test returned 3.5 instead of 2.23m for distance from speaker apriltag.
         */
        double distance = tag_pose.getY() - getTranslation().getY();

        Logger.recordOutput("Speaker Distance", distance);
        Logger.recordOutput("Robot-Translation", getTranslation());
    }
}
