package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;

public class SwerveConstants {

    // PID Values for the Steering Motors
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(70).withKI(0).withKD(0.1)
            .withKS(0.1).withKV(1.91).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // PID Values for the Drive Motors
    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.125);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // Drive Motor Stator Limit
    private static final Current kSlipCurrent = Amps.of(80.0);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Amps.of(40))
                            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

    public static final LinearVelocity kTranslationSpeedAt12Volts = FeetPerSecond.of(16.16);
    public static final AngularVelocity kRotationSpeedAt12Volts = DegreesPerSecond.of(650);

    private static final double kCoupleRatio = 54d / 12;

    private static final double kDriveGearRatio = 6.48;
    private static final double kSteerGearRatio = 12.1;

    private static final Distance kWheelRadius = Inches.of(2.167);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = false;

    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kTranslationSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // NOTE: X2 Modules have a 2.5" Side Offset. For a 30x30 robot the front left
    // module is at (30/2 - 2.5, 30/2 - 2.5)

    private static final double kFrameWidth = 29; // Y
    private static final double kFrameLength = 29; // X

    // Pigeon
    private static final int kPigeonId = 13;

    // Front Left
    private static final int kFrontLeftDriveMotorId = 7;
    private static final int kFrontLeftSteerMotorId = 8;
    private static final int kFrontLeftEncoderId = 12;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.353 + 3d/8 + 1d/2);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(kFrameLength / 2 - 2.5);
    private static final Distance kFrontLeftYPos = Inches.of(kFrameWidth / 2 - 2.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 1;
    private static final int kFrontRightSteerMotorId = 2;
    private static final int kFrontRightEncoderId = 9;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.238 + 1d/8 + 1d/2);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(kFrameLength / 2 - 2.5);
    private static final Distance kFrontRightYPos = Inches.of(-kFrameWidth / 2 + 2.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 11;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.491 - 3d/8 + 1d/2);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-kFrameLength / 2 + 2.5);
    private static final Distance kBackLeftYPos = Inches.of(kFrameWidth / 2 - 2.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 3;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 10;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.3459 - 1d/8  + 1d/2);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-kFrameLength / 2 + 2.5);
    private static final Distance kBackRightYPos = Inches.of(-kFrameWidth / 2 + 2.5);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
            .createModuleConstants(
                    kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                    kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted,
                    kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
            .createModuleConstants(
                    kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                    kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted,
                    kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
            .createModuleConstants(
                    kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                    kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted,
                    kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
            .createModuleConstants(
                    kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                    kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted,
                    kBackRightEncoderInverted);

    public static Swerve createDrivetrain() {
        return new Swerve(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static class CTRESwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public CTRESwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules);
        }
    }
}