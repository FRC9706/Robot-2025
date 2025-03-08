package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.FeetPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {

  // Drivetrain Subsystem
  private final Swerve drivetrain = SwerveConstants.createDrivetrain();
  private final AutoFactory autofact;
  private final Limelight limelight = new Limelight();
  private final Arm arm = new Arm();

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(0);

  // Theta Controller lastvals, janky solution but im still praying it works
  private double valx;
  private double LastValY = 0;
  private double LastValX = 0;
  private double valy;
  private double getLastThetaControllerInputX() {
    valx = driverController.getRightX();
    if (Math.abs(valx) < 0.05) {
      return LastValX;
    } else {
      LastValX = valx;
      return valx;
    }
  }
  private double getLastThetaControllerInputY() {
    valy = driverController.getRightY();
    if (Math.abs(valy) < 0.05) {
      return LastValY;
    } else {
      LastValY = valy;
      return valy;
    }
  }
  /**
   * 
   */
  public Robot() {

    // Configure DogLog
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureConsole(true)
    );

    autofact = new AutoFactory(
      () -> drivetrain.getState().Pose,
      drivetrain::resetPose, 
      drivetrain::followTrajectory, 
      true,
      drivetrain);

      driverController.x().whileTrue(
        Commands.run(() -> {
            // Keep moving
            drivetrain.goToLimelight(driverController::getLeftX);
        }, drivetrain
        ));

      driverController.x().onFalse(
        Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(0).withVelocityX(0)))
      );

    

    //Teleop Speed Multipliers. Percentages of the max speed.
    double translationSpeedMultiplier = 0.25;
    double controllerDeadband = 0.1;

    new Rotation2d();
    SlewRateLimiter targetDirectionLimiter = new SlewRateLimiter(Math.PI);
    // Drive command
    final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(SwerveConstants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * controllerDeadband * translationSpeedMultiplier)
      .withDriveRequestType(DriveRequestType.Velocity);
    snapDrive.HeadingController = new PhoenixPIDController(SwerveConstants.HeadingControlkP, SwerveConstants.HeadingControlkI, SwerveConstants.HeadingControlkD);
    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the default command for the drivetrain to be the teleop drive command.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> snapDrive
              .withVelocityX(-driverController.getLeftY() * SwerveConstants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * translationSpeedMultiplier)
              .withVelocityY(-driverController.getLeftX() * SwerveConstants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * translationSpeedMultiplier)
              .withTargetDirection(Rotation2d.fromRadians(targetDirectionLimiter.calculate((Math.atan2(getLastThetaControllerInputY(), -getLastThetaControllerInputX()) + Math.PI/2))))
              .withTargetRateFeedforward(SwerveConstants.HeadingFF)
              )
          /*   () -> new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(SwerveConstants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * controllerDeadband * translationSpeedMultiplier)
                .withRotationalDeadband(SwerveConstants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * controllerDeadband * rotationSpeedMultiplier)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-driverController.getLeftY() * SwerveConstants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * translationSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * SwerveConstants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * translationSpeedMultiplier)
                .withTargetDirection(Rotation2d.fromDegrees(NonZeroRad(-driverController.getRightX(), -driverController.getRightY())))
                .withTargetRateFeedforward(SwerveConstants.HeadingFF)
                .withHeadingPID(SwerveConstants.HeadingControlkP, SwerveConstants.HeadingControlkI, SwerveConstants.HeadingControlkD)
                // .withRotationa
                lRate(-driverController.getRightX() * 0.2 * SwerveConstants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * rotationSpeedMultiplier)
            */
            
        );
      

    // Button to reset the field-relative rotation to 0 degrees. Face the robot away
    // from the driver station wall when pressing.
    driverController.a().onTrue(
        new ConditionalCommand(
            Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero), drivetrain),
            Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.k180deg), drivetrain),
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
              return alliance == Alliance.Blue;
            }
        )
    );

    driverController.leftTrigger().onTrue(
      Commands.run(() -> 
        arm.goToPosition1()
    ));

    driverController.rightTrigger().onTrue(
      Commands.run(() -> 
        arm.goToPosition2()
    ));

  }

  @Override
  public void autonomousInit() {
    // Basic taxi auto.
    // Drives forward at 2 m/s for 1 second.
    TestAuto().cmd().schedule();
    // drivetrain.applyRequest(
    //   () -> new SwerveRequest.RobotCentric()
    //   .withVelocityX(2)
    //   .withVelocityY(0)
    //   .withRotationalRate(0)
    // ).withTimeout(1).schedule();
  }

  @Override
  public void robotInit() {
    // Set the Limelight to the AprilTag pipeline
    limelight.setAprilTagPipeline();
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

  public AutoRoutine TestAuto() {
    AutoRoutine routine = autofact.newRoutine("test");

    // Load the routine's trajectories
    AutoTrajectory reefTraj = routine.trajectory("GoToClosestReef", 0);

    // When the routine begins, reset odometry and start the first trajectory 
    routine.active().onTrue(
        Commands.sequence(
            reefTraj.resetOdometry(),
            reefTraj.cmd()
        )
    );
    return routine;
}

}
