package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private static final double kP_steering = 0.05;
    private static final double kP_distance = 0.05;

    private final NetworkTable limelightTable;

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Get for tx horizontal offset
    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    // Get for ty (vert offset)
    public double getTy() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    // Get for ta (target area)
    public double getTa() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    // if the tag is visible or not
    public boolean isTagThere() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    // find steering adjustment based on tx
    public double getSteerAdj() {
        double tx = getTx();
        return kP_steering * tx;
    }

    // find distance adjustment based on ty
    public double getDistanceAdj() {
        double ty = getTy();
        return kP_distance * ty;
    }
}
