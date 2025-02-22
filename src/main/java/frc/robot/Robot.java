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
import frc.robot.subsystems.SwerveConstants;

public class Robot extends TimedRobot {

  // Drivetrain Subsystem
  private final Swerve drivetrain = SwerveConstants.createDrivetrain();
  private final AutoFactory autofact;

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(0);


  private double NonZeroRad(double x, double y) {
    double radians;
    radians = Math.atan2(y, x);
    return radians;
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

    //Teleop Speed Multipliers. Percentages of the max speed.
    double translationSpeedMultiplier = 0.25;
    double rotationSpeedMultiplier = 0.25;
    double controllerDeadband = 0.1;

    new Rotation2d();
    SlewRateLimiter targetDirectionLimiter = new SlewRateLimiter(Math.PI/2);
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
              .withTargetDirection(Rotation2d.fromRadians(targetDirectionLimiter.calculate((Math.atan2(driverController.getRightY(), -driverController.getRightX()) + Math.PI/2))))
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
                // .withRotationalRate(-driverController.getRightX() * 0.2 * SwerveConstants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * rotationSpeedMultiplier)
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
