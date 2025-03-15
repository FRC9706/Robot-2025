package frc.robot;

public class ArmConstants {
    public static final int kArmMotorID = 13;
    public static final double kArmPgain = 0.0001;
    public static final double kArmIgain = 0;
    public static final double kArmDgain = 0;
    public static final double kArmPositionConversionFactor = 64/12;
    public static final double kArmVelocityConversionFactor = 64/12;
    public static final boolean kInverted = false;
    public static final double kArmPosition1 = 10;
    public static final double kArmPosition2 = 5;
    public static final double kMinOutput = -0.1;
    public static final double kMaxOutput = 0.1;
    public static boolean isOn = false;
}
