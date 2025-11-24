package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import dev.doglog.DogLog;

import java.util.ArrayList;
import java.util.List;

import com.alibaba.fastjson2.annotation.JSONType;

public class Limelight {

    // ------------------------------
    // NetworkTable for Limelight data
    // ------------------------------
    private final static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // ------------------------------
    // calibration parameters
    // ------------------------------
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double[] ROBOT_TO_TAG_TRANSLATION_INCHES = {12.71654, 0.0, 8.30708661};
    public static final double TAG_PITCH_DEGREES = 10.9;
    public static final double TAG_YAW_DEGREES = 0.0;
    public static final double TAG_ROLL_DEGREES = 0.0;

    // ------------------------------
    // Constants for pose filtering
    // ------------------------------
    public static final double MAX_AMBIGUITY = 0.3;        // Max ambiguity to accept pose
    public static final double MAX_Z_ERROR = 0.75;         // Max vertical error in meters

    // ------------------------------
    // Nested class holding calibration info & unit conversions
    // ------------------------------
    @JSONType(deserializeFeatures = com.alibaba.fastjson2.JSONReader.Feature.FieldBased)
    public static class PoseCalibrationInfo {
        public String limelightName = LIMELIGHT_NAME;
        public double[] robotToTagTranslationInches = ROBOT_TO_TAG_TRANSLATION_INCHES;

        public Pose3d getRobotPoseInTagSpaceMeters() {
            return new Pose3d(
                Units.inchesToMeters(robotToTagTranslationInches[0]),
                Units.inchesToMeters(robotToTagTranslationInches[1]),
                -Units.inchesToMeters(robotToTagTranslationInches[2]),
                new Rotation3d(
                    Units.degreesToRadians(TAG_PITCH_DEGREES),
                    Units.degreesToRadians(TAG_YAW_DEGREES),
                    Units.degreesToRadians(TAG_ROLL_DEGREES + 180.0)
                ));
        }
    }

    private PoseCalibrationInfo calibrationInfo = new PoseCalibrationInfo();

    // ------------------------------
    // Pose collection for calibration averaging
    // ------------------------------
    private final List<Pose3d> limelightPoseMeasurements = new ArrayList<>();
    private boolean didWriteResults = false;

    // ------------------------------
    // Basic Limelight methods
    // ------------------------------
    public void setAprilTagPipeline() {
        limelightTable.getEntry("pipeline").setNumber(DetectorConstants.kAprilTagPiplineNumber);
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

    // ------------------------------
    // Logging Limelight basic info to DogLog and NetworkTables
    // ------------------------------
    public void logLimelightStats() {
        double totalLatency = limelightTable.getEntry("tl").getDouble(0);
        double tA = limelightTable.getEntry("ta").getDouble(0);
        double tX = limelightTable.getEntry("tx").getDouble(0);
        double tY = limelightTable.getEntry("ty").getDouble(0);
        double tV = limelightTable.getEntry("tv").getDouble(0);

        DogLog.log("Limelight/TotalLatency", totalLatency);
        DogLog.log("Limelight/tA", tA);
        DogLog.log("Limelight/tX", tX);
        DogLog.log("Limelight/tY", tY);
        DogLog.log("Limelight/tV", tV);

        limelightTable.getEntry("totalLatency").setDouble(totalLatency);
        limelightTable.getEntry("log_tA").setDouble(tA);
        limelightTable.getEntry("log_tX").setDouble(tX);
        limelightTable.getEntry("log_tY").setDouble(tY);
        limelightTable.getEntry("log_tV").setDouble(tV);
    }

    // ------------------------------
    // Converts Limelight tag-space pose array to robot pose in meters with rotation
    // The tagSpacePose array is expected as [x, y, z, roll, pitch, yaw] or equivalent
    // ------------------------------
    private static final Pose3d limelightTagSpaceToOurTagSpace(double[] tagSpacePose) {
        return new Pose3d(
            -tagSpacePose[2],
            tagSpacePose[0],
            -tagSpacePose[1],
            new Rotation3d(
                Units.degreesToRadians(tagSpacePose[5]),
                Units.degreesToRadians(tagSpacePose[3]),
                -Units.degreesToRadians(tagSpacePose[4] + 180)
            ));
    }

    // ------------------------------
    // Validates pose quality based on ambiguity and Z error filters
    // Returns true if pose passes filters
    // ------------------------------
    public boolean isValidPose(double ambiguity, Pose3d pose) {
        return ambiguity <= MAX_AMBIGUITY && Math.abs(pose.getZ()) <= MAX_Z_ERROR;
    }

    // ------------------------------
    // Start or reset the calibration pose accumulation
    // ------------------------------
    public void startCalibration() {
        limelightPoseMeasurements.clear();
        didWriteResults = false;
    }

    /**
     * Adds a new pose measurement for calibration if valid.
     * @param cameraPoseTagSpace 6-element array pose from Limelight (position and rotation degrees).
     * @param ambiguity Vision ambiguity metric from Limelight.
     */
    public void updateCalibration(double[] cameraPoseTagSpace, double ambiguity) {
        Pose3d cameraPose = limelightTagSpaceToOurTagSpace(cameraPoseTagSpace);

        if (!isValidPose(ambiguity, cameraPose)) {
            // Discard poor quality measurements early
            return;
        }

        limelightPoseMeasurements.add(cameraPose);

        // When enough data is collected, calculate and output calibration result
        if (limelightPoseMeasurements.size() >= 3000 && !didWriteResults) {
            Pose3d averagePose = getAverageLimelightPose();
            Pose3d robotPose = calibrationInfo.getRobotPoseInTagSpaceMeters();
            Pose3d limelightRelativeToRobot = averagePose.relativeTo(robotPose);

            System.out.println(String.format(
                "Calibration complete for %s! Limelight pose relative to robot (Forward, Right, Up, Roll, Pitch, Yaw): " +
                "(%f, %f, %f, %f, %f, %f)",
                calibrationInfo.limelightName,
                limelightRelativeToRobot.getX(),
                -limelightRelativeToRobot.getY(),
                limelightRelativeToRobot.getZ(),
                Units.radiansToDegrees(limelightRelativeToRobot.getRotation().getX()),
                Units.radiansToDegrees(limelightRelativeToRobot.getRotation().getY()),
                Units.radiansToDegrees(limelightRelativeToRobot.getRotation().getZ())
            ));

            didWriteResults = true;
        }
    }

    // ------------------------------
    // Computes the average of all accumulated pose measurements
    // ------------------------------
    private Pose3d getAverageLimelightPose() {
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;

        for (Pose3d pose : limelightPoseMeasurements) {
            x += pose.getX();
            y += pose.getY();
            z += pose.getZ();
            roll += pose.getRotation().getX();
            pitch += pose.getRotation().getY();
            yaw += pose.getRotation().getZ();
        }

        int count = limelightPoseMeasurements.size();
        return new Pose3d(
            x / count,
            y / count,
            z / count,
            new Rotation3d(roll / count, pitch / count, yaw / count)
        );
    }
    
}