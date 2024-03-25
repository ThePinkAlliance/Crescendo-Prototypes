package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and
 * {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class FieldConstants {
    public static final boolean isWPIField = false; // Red alliance

    public static double fieldLength = Units.inchesToMeters(651.223);
    public static double fieldWidth = Units.inchesToMeters(323.277);
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    // AprilTag constants
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);
    public static final AprilTagFieldLayout aprilTags = k2024Crescendo.loadAprilTagLayoutField();
}
