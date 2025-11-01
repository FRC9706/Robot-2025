package frc.robot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.fasterxml.jackson.annotation.JsonFormat.Feature;

import dev.doglog.DogLog;

public class Limelight {

    private final static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
        public void setAprilTagPipeline() {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(DetectorConstants.kAprilTagPiplineNumber);  // Change '1' to your AprilTag pipeline number
        }
    
        public static double getA() {
            return limelightTable.getEntry("ta").getDouble(0.0);
        }
        public static double getX() {
            return limelightTable.getEntry("tx").getDouble(0.0);
        }

        public static double getY() {
            return limelightTable.getEntry("ty").getDouble(0.0);
        }

        public static double getV() {
            return limelightTable.getEntry("tv").getDouble(0.0);
        }

    public void logLimelightStats() {
        // Read values from Limelight NetworkTables
        double totalLatency = limelightTable.getEntry("tl").getDouble(0); // Total Latency (ms)
        double tA = limelightTable.getEntry("ta").getDouble(0);  // Target Area
        double tX = limelightTable.getEntry("tx").getDouble(0);  // Horizontal Offset
        double tY = limelightTable.getEntry("ty").getDouble(0);  // Vertical Offset
        double tV = limelightTable.getEntry("tv").getDouble(0);  // Valid Target (0 or 1)
        // Log to DogLog
        DogLog.log("Limelight/TotalLatency", totalLatency);
        DogLog.log("Limelight/tA", tA);
        DogLog.log("Limelight/tX", tX);
        DogLog.log("Limelight/tY", tY);
        DogLog.log("Limelight/tV", tV);
        // Log to NetworkTables
        limelightTable.getEntry("totalLatency").setDouble(totalLatency);
        limelightTable.getEntry("log_tA").setDouble(tA);
        limelightTable.getEntry("log_tX").setDouble(tX);
        limelightTable.getEntry("log_tY").setDouble(tY);
        limelightTable.getEntry("log_tV").setDouble(tV);
    }

    // @JSONType(deserializeFeatures = Feature.FieldBased)
    // public static class PoseCalibrationInfo {
    //     public String limelightName;
    //     // Assumes that the tag yaw, pitch, and roll are zero
    //     public double[] robotToTagTranslationInches;

    //     PoseCalibrationInfo() {
    //         robotToTagTranslationInches = new double[3];
    //     }

    //     public Pose3d getRobotPoseInTagSpaceMeters() {
    //         return new Pose3d(Units.inchesToMeters(robotToTagTranslationInches[0]), Units.inchesToMeters(robotToTagTranslationInches[1]), 
    //                 -Units.inchesToMeters(robotToTagTranslationInches[2]), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));
    //     }
    // }

        /**
     * The limelight defines tag space as the following:
     * +x points to the right of the target (if looking at the target)
     * +y point downward
     * +z points out of the tag
     * 
     * We want to convert this to something more similar to robot space:
     * +x points out of the tag
     * +y points to the left of the tag if you embody the tag (to the right if you lopok at the tag)
     * +z points upward
     */
    private static final Pose3d limelightTagSpaceToOurTagSpace(double[] tagSpacePose) {
        return new Pose3d(-tagSpacePose[2], tagSpacePose[0], -tagSpacePose[1],
                new Rotation3d(Units.degreesToRadians(tagSpacePose[5]), Units.degreesToRadians(tagSpacePose[3]), 
                -Units.degreesToRadians(tagSpacePose[4] +  180)));
    }



}