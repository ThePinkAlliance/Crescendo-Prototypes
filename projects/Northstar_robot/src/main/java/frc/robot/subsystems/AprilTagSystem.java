package frc.robot.subsystems;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Alert.AlertType;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagSystem extends SubsystemBase {
    // was 0.15 but .55 for debug
    private static final double ambiguityThreshold = 0.20;
    private static final double targetLogTimeSecs = 0.1;
    private static final double fieldBorderMargin = 0.5;
    private static final double zMargin = 0.75;
    private static final Pose3d[] cameraPoses;
    private static final double xyStdDevCoefficient;
    private static final double thetaStdDevCoefficient;

    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIO.AprilTagVisionIOInputs[] inputs;

    private boolean enableVisionUpdates = true;
    private Alert enableVisionUpdatesAlert = new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);
    private Consumer<List<PoseEstimator.TimestampedVisionUpdate>> visionConsumer = (x) -> {
    };
    private Supplier<Pose2d> poseSupplier = () -> new Pose2d(1.5, 0.0, new Rotation2d());
    private Map<Integer, Double> lastFrameTimes = new HashMap<>();
    private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
    private Pose3d demoTagPose = null;

    static {
        // Front left (forward facing, camera 6)
        cameraPoses = new Pose3d[] {
                new Pose3d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(0),
                        new Rotation3d(0.0, Units.degreesToRadians(0), 0.0)
                                .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(18), Units.degreesToRadians(0.0))))
        };
        xyStdDevCoefficient = 0.01;
        thetaStdDevCoefficient = 0.01;
    }

    public AprilTagSystem(AprilTagVisionIO... io) {
        System.out.println("[Init] Creating AprilTagVision");
        this.io = io;
        inputs = new AprilTagVisionIO.AprilTagVisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagVisionIO.AprilTagVisionIOInputs();
        }

        // Create map of last frame times for instances
        for (int i = 0; i < io.length; i++) {
            lastFrameTimes.put(i, 0.0);
        }

        // Create map of last detection times for tags
        FieldConstants.aprilTags
                .getTags()
                .forEach(
                        (AprilTag tag) -> {
                            lastTagDetectionTimes.put(tag.ID, 0.0);
                        });
    }

    public void setDataInterfaces(
            Consumer<List<PoseEstimator.TimestampedVisionUpdate>> visionConsumer, Supplier<Pose2d> poseSupplier) {
        this.visionConsumer = visionConsumer;
        this.poseSupplier = poseSupplier;
    }

    /** Returns the field relative pose of the demo tag. */
    public Optional<Pose3d> getDemoTagPose() {
        return Optional.ofNullable(demoTagPose);
    }

    public Supplier<Pose2d> getRobotPose() {
        return this.poseSupplier;
    }

    /** Sets whether vision updates for odometry are enabled. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        enableVisionUpdates = enabled;
        enableVisionUpdatesAlert.set(!enabled);
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
        }

        // Loop over instances
        List<Pose2d> allRobotPoses = new ArrayList<>();
        List<Pose3d> allRobotPoses3d = new ArrayList<>();
        List<PoseEstimator.TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
        for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {

            // Loop over frames
            for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
                lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
                var timestamp = inputs[instanceIndex].timestamps[frameIndex];
                var values = inputs[instanceIndex].frames[frameIndex];

                // Exit if blank frame
                if (values.length == 0 || values[0] == 0) {
                    continue;
                }

                // Switch based on number of poses
                Pose3d cameraPose = null;
                Pose3d robotPose3d = null;
                switch ((int) values[0]) {
                    case 1:
                        // One pose (multi-tag), use directly
                        cameraPose = new Pose3d(
                                values[2],
                                values[3],
                                values[4],
                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                        robotPose3d = cameraPose.transformBy(
                                GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());
                        break;

                    case 2:
                        // Two poses (one tag), disambiguate
                        double error0 = values[1];
                        double error1 = values[9];

                        Pose3d cameraPose0 = new Pose3d(
                                values[2],
                                values[3],
                                values[4],
                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])).times(-1));
                        Pose3d cameraPose1 = new Pose3d(
                                values[10],
                                values[11],
                                values[12],
                                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16]))
                                        .times(-1));
                        Pose3d robotPose3d0 = cameraPose0.transformBy(
                                GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());
                        Pose3d robotPose3d1 = cameraPose1.transformBy(
                                GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());

                        // Select pose using projection errors
                        if (error0 < error1 * ambiguityThreshold) {
                            cameraPose = cameraPose0;
                            robotPose3d = robotPose3d0;
                        } else if (error1 < error0 * ambiguityThreshold) {
                            cameraPose = cameraPose1;
                            robotPose3d = robotPose3d1;
                        }
                        break;
                }

                // Exit if no data
                if (cameraPose == null || robotPose3d == null) {
                    continue;
                }

                // Exit if robot pose is off the field
                // if (robotPose3d.getX() < -fieldBorderMargin
                // || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
                // || robotPose3d.getY() < -fieldBorderMargin
                // || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
                // || robotPose3d.getZ() < -zMargin
                // || robotPose3d.getZ() > zMargin) {
                // continue;
                // }

                // Get 2D robot pose
                Pose2d robotPose = robotPose3d.toPose2d();

                Logger.recordOutput("AprilTagVision/Camera", cameraPose);

                // Get tag poses and update last detection times
                List<Pose3d> tagPoses = new ArrayList<>();
                for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
                    int tagId = (int) values[i];
                    lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                    Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) values[i]);
                    if (tagPose.isPresent()) {
                        tagPoses.add(tagPose.get());
                    }
                }

                // Calculate average distance to tag
                double totalDistance = 0.0;
                for (Pose3d tagPose : tagPoses) {
                    totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                }
                double avgDistance = totalDistance / tagPoses.size();

                // Add to vision updates
                double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
                double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
                visionUpdates.add(
                        new PoseEstimator.TimestampedVisionUpdate(
                                timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
                allRobotPoses.add(robotPose);
                allRobotPoses3d.add(robotPose3d);

                // Log data from instance
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/LatencySecs",
                                Timer.getFPGATimestamp() - timestamp);
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose",
                                robotPose);
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose3d",
                                robotPose3d);
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            }

            // If no frames from instances, clear robot pose
            if (inputs[instanceIndex].timestamps.length == 0) {
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose",
                                new double[] {});
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose3d",
                                new double[] {});
            }

            // If no recent frames from instance, clear tag poses
            if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
                Logger
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                                new double[] {});
            }
        }

        // Log robot poses
        Logger
                .recordOutput(
                        "AprilTagVision/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
        Logger
                .recordOutput(
                        "AprilTagVision/RobotPoses3d",
                        allRobotPoses3d.toArray(new Pose3d[allRobotPoses3d.size()]));

        // Log tag poses
        List<Pose3d> allTagPoses = new ArrayList<>();
        for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
            SmartDashboard.putNumber("detEntry", Timer.getFPGATimestamp() - detectionEntry.getValue());
            if ((Timer.getFPGATimestamp() - detectionEntry.getValue()) < targetLogTimeSecs) {
                Optional<Pose3d> pose = FieldConstants.aprilTags.getTagPose(detectionEntry.getKey());

                if (pose.isPresent()) {
                    allTagPoses.add(pose.get());
                }
            }
        }
        Logger
                .recordOutput(
                        "AprilTagVision/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

        // Log demo tag pose
        if (demoTagPose == null) {
            Logger.recordOutput("AprilTagVision/DemoTagPose", new double[] {});
        } else {
            Logger.recordOutput("AprilTagVision/DemoTagPose", demoTagPose);
        }
        Logger.recordOutput("AprilTagVision/DemoTagPoseId", new long[] { 29 });

        // Send results to pose esimator
        if (enableVisionUpdates) {
            visionConsumer.accept(visionUpdates);
        }
    }
}