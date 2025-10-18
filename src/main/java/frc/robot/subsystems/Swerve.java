package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DetectorConstants;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;
//import frc.robot.LimelightHelpers;
// import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Parameters.CTRESwerveDrivetrain;

public class Swerve extends CTRESwerveDrivetrain implements Subsystem {

    // Rotation values to correctly flip field-relative controls for the driver
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;
    // private static final Pigeon2 pigeon = new Pigeon2(SwerveConstants.kPigeonId, "canivore");

    // Subsystem Constructor
    public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, 250, modules);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /*
     * Send a request to the drivetrain to set the control state.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command applyRequestOnce(Supplier<SwerveRequest> requestSupplier) {
        return runOnce(() -> this.setControl(requestSupplier.get()));
    }

    // public void goToAngle(double DesiredAngle, Swerve drivebase) {
    //     drivebase.applyRequest(() -> new SwerveRequest.FieldCentricFacingAngle()
    //                                                 .withCenterOfRotation(Translation2d.kZero)
    //                                                 .withVelocityX(5)
    //                                                 .withVelocityY(0)
    //                                                 .withHeadingPID(SwerveConstants.HeadingControlkP, SwerveConstants.HeadingControlkI, SwerveConstants.HeadingControlkD)
    //                                                 .withTargetDirection(Rotation2d.fromDegrees(DesiredAngle)));

    // }

    @Override
    public void periodic() {

        // Applies the correct rotation for the driver based on the alliance color
        // This is locked in for the first enable after the robot boots.
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        // Logging

        //Log General Swerve Information
        DogLog.log("Swerve/ModuleStates", getState().ModuleStates);
        DogLog.log("Swerve/ModuleStateSetpoints", getState().ModuleTargets);
        DogLog.log("Swerve/OdometryPose", getState().Pose);
        DogLog.log("Swerve/ChassisSpeeds", getState().Speeds);

        // Module Name Keys
        String[] moduleNames = new String[] { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };

        // Log Module Data
        for (int i = 0; i < 4; i++) {
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/EncoderAbsolutePosition", getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/EncoderPosition", getModule(i).getEncoder().getPosition().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/DriveVelocity", getModule(i).getCurrentState().speedMetersPerSecond);
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/DriveVelocitySetpoint", getModule(i).getTargetState().speedMetersPerSecond);
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/DriveSupplyCurrent", getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/DriveStatorCurrent", getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/DriveVoltage", getModule(i).getDriveMotor().get() * RobotController.getBatteryVoltage());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/DriveTemperature", getModule(i).getDriveMotor().getDeviceTemp().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/TurnPosition", getModule(i).getCurrentState().angle.getRadians());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/TurnPositionSetpoint", getModule(i).getTargetState().angle.getRadians());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/TurnSupplyCurrent", getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/TurnStatorCurrent", getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/TurnVoltage", getModule(i).getSteerMotor().get() * RobotController.getBatteryVoltage());
            DogLog.log("Swerve/Modules/" + moduleNames[i] + "/TurnTemperature", getModule(i).getSteerMotor().getDeviceTemp().getValueAsDouble());
        }

    }

    private final PIDController xController = new PIDController(10, 0.0, 0);
    private final PIDController yController = new PIDController(10, 0.0, 0);
    private final PIDController thetaController = new PIDController(7.5, 0.0, 0);

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getState().Pose;

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getRadians(), sample.heading)
        ); 
        this.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
        }

    // Simulation variables
    private static final double kSimLoopPeriod = 0.005;
    private Notifier simNotifier = null;
    private double lastSimTime;

    // Simulation Thread
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }
    
    
    // Constants for PID tuning and target positioning
    private static final double kPPosition = 0.05;
    private static final double kPYaw = 0.02;
    private static final double targetAreaSetpoint = 10.0; // Example area for stopping near tag

    // public void goToAprilTag() {
    //     // Get Limelight target data from network tables
    //     boolean targetVisible = LimelightHelpers.getTV("limelight");
    //     double tx = LimelightHelpers.getTX("limelight"); // Horizontal offset (degrees)
    //     double ty = LimelightHelpers.getTY("limelight"); // Vertical offset (degrees)
    //     double ta = LimelightHelpers.getTA("limelight"); // Target area (proxy for distance)

    //     if (!targetVisible) {
    //         // No target found, stop robot safely
    //         this.setControl(new SwerveRequest.FieldCentric()
    //             .withVelocityX(0)
    //             .withVelocityY(0)
    //             .withRotationalRate(0));
    //         return;
    //     }

    //     // Forward/backward control based on target area error
    //     double forwardCommand = (targetAreaSetpoint - ta) * kPPosition;
    //     forwardCommand = Math.max(-1, Math.min(1, forwardCommand)); // Clamp

    //     // Left/right strafing based on horizontal offset (tx)
    //     double strafeCommand = tx * kPPosition;
    //     strafeCommand = Math.max(-1, Math.min(1, strafeCommand)); // Clamp

    //     // Rotational control based on horizontal offset (tx) to align robot yaw
    //     double rotateCommand = tx * kPYaw;
    //     rotateCommand = Math.max(-1, Math.min(1, rotateCommand)); // Clamp

    //     // Apply commands safely to drivetrain (field-centric)
    //     this.setControl(new SwerveRequest.FieldCentric()
    //         .withVelocityX(forwardCommand)
    //         .withVelocityY(-strafeCommand) // Invert if necessary based on your orientation
    //         .withRotationalRate(-rotateCommand)); // Invert if necessary for your setup
    // }

    




    public void goToAprilTag() {
    // Get Limelight readings
    double tv = Limelight.getV(); // Valid target (0 = no, 1 = yes)
    double tx = Limelight.getX(); // Horizontal offset (+ = right)
    double ty = Limelight.getY(); // Vertical offset (+ = up)
    double ta = Limelight.getA(); // Target area (indicator for distance)
    
    // Tuning constants
    double kPforward = 0.05;   // Forward control
    double kPstrafe = 0.035;   // Side-to-side correction
    double kProtation = 0.02;  // rotational adjustment
    double areaTarget = 10.0;   // tag area for stopping distance
    
    // Deadbands
    double txDeadband = 1.0;
    double tyDeadband = 1.0;

    if (tv < 1.0) {
        // No tag detected
        System.out.println("No AprilTag found just like your father.");
        this.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        return;
    }

    // Proportional forward control using area (closer = larger ta)
    double forwardCommand = (areaTarget - ta) * kPforward;
    if (Math.abs(areaTarget - ta) < 0.2) forwardCommand = 0; // stop near target
    
    // Horizontal centering correction
    double strafeCommand = 0;
    if (Math.abs(tx) > txDeadband) strafeCommand = tx * kPstrafe;

    // rotational control to ensure not angled relative to tag
    double rotateCommand = 0;
    if (Math.abs(tx) > txDeadband) rotateCommand = tx * kProtation;
    
    // Clamp speeds to prevent aggressive jumping
    forwardCommand = Math.max(-1, Math.min(1, forwardCommand));
    strafeCommand = Math.max(-1, Math.min(1, strafeCommand));
    rotateCommand = Math.max(-1, Math.min(1, rotateCommand));

    // Apply movement control
    this.setControl(new SwerveRequest.FieldCentric()
        .withVelocityX(forwardCommand)
        .withVelocityY(-strafeCommand)
        .withRotationalRate(-rotateCommand));
    }


//     // Limelight Variables
//     private boolean v;
//     private double x;
//     private double y;

//     public void goToLimelight() {
//         boolean v = LimelightHelpers.getTV(DetectorConstants.kLimelightName);
//         double tx = LimelightHelpers.getTX(DetectorConstants.kLimelightName);
//         double ty = LimelightHelpers.getTY(DetectorConstants.kLimelightName);
    
//         double kP = 0.06;
    
//         if (v) {
//             // Move toward the target
//             this.setControl(new SwerveRequest.FieldCentric()
//                 .withVelocityX(ty * kP)  // Forward/backward correction
//                 .withVelocityY(tx * kP)  // Left/right correction
//                 .withRotationalRate(0));
//         } else {
//             // No target found
//             System.out.println("BRO WHERE IS THE april tag");
//             this.setControl(new SwerveRequest.FieldCentric()
//                 .withVelocityX(0)
//                 .withVelocityY(0)
//                 .withRotationalRate(0));
//         }
//     }    
 };
