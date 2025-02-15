package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveConstants;

public class Robot extends TimedRobot {

  // Drivetrain Subsystem
  private final Swerve drivetrain = SwerveConstants.createDrivetrain();

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(0);

  public Robot() {

    // Configure DogLog
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureConsole(true)
    );

    //Teleop Speed Multipliers. Percentages of the max speed.
    double translationSpeedMultiplier = 0.25;
    double rotationSpeedMultiplier = 0.25;
    double controllerDeadband = 0.1;

    // Set the default command for the drivetrain to be the teleop drive command.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.kTranslationSpeedAt12Volts.in(MetersPerSecond) * controllerDeadband * translationSpeedMultiplier)
                .withRotationalDeadband(SwerveConstants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * controllerDeadband * rotationSpeedMultiplier)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-driverController.getLeftY() * SwerveConstants.kTranslationSpeedAt12Volts.in(MetersPerSecond) * translationSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * SwerveConstants.kTranslationSpeedAt12Volts.in(MetersPerSecond) * translationSpeedMultiplier)
                .withRotationalRate(-driverController.getRightX() * SwerveConstants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * rotationSpeedMultiplier)
        )
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
    drivetrain.applyRequest(
      () -> new SwerveRequest.RobotCentric()
      .withVelocityX(2)
      .withVelocityY(0)
      .withRotationalRate(0)
    ).withTimeout(1).schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

}
