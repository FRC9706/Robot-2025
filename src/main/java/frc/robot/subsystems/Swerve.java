package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveConstants.CTRESwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;

public class Swerve extends CTRESwerveDrivetrain implements Subsystem {

    // Rotation values to correctly flip field-relative controls for the driver
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

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
}
