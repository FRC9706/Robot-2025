package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DetectorConstants;
import dev.doglog.DogLog;

public class Limelight {

    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public void setAprilTagPipeline() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(DetectorConstants.kAprilTagPiplineNumber);  // Change '1' to your AprilTag pipeline number
    }

    public double getA() {
        return limelightTable.getEntry("ta").getDouble(0.0);
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
}
