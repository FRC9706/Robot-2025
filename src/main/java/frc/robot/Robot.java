package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import static edu.wpi.first.units.Units.FeetPerSecond;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intout;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Autos;

public class Robot extends TimedRobot {

  // initialize subsystems
  private final Swerve drivetrain = Parameters.createDrivetrain();
  private final Arm arm = Arm.getInstance();
  // private final Intout intout = new Intout();
  private final Climb climb = new Climb();
  private final Autos autos = new Autos();

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(0);

  public Robot() {

    // Configure DogLog
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureConsole(true)
    );

      driverController.x().onFalse(
        Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withRotationalRate(0).withVelocityX(0)))
      );

    

    // Teleop Speed Multipliers. Percentages of the max speed. 
    double translationSpeedMultiplier = 1;
    double controllerDeadband = 0.1;

    new Rotation2d();
    // Drive command
    final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(Parameters.kTranslationSpeedAt12Volts.in(FeetPerSecond) * controllerDeadband * translationSpeedMultiplier)
      .withDriveRequestType(DriveRequestType.Velocity);
    snapDrive.HeadingController = new PhoenixPIDController(Parameters.HeadingControlkP, Parameters.HeadingControlkI, Parameters.HeadingControlkD);
    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the default command for the drivetrain to be the teleop drive command.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
        .withDeadband(Parameters.kTranslationSpeedAt12Volts.in(MetersPerSecond) * controllerDeadband * translationSpeedMultiplier)
        .withRotationalDeadband(Parameters.kRotationSpeedAt12Volts.in(RadiansPerSecond) * controllerDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(-driverController.getLeftY() * Parameters.kTranslationSpeedAt12Volts.in(MetersPerSecond) * translationSpeedMultiplier)
        .withVelocityY(-driverController.getLeftX() * Parameters.kTranslationSpeedAt12Volts.in(MetersPerSecond) * translationSpeedMultiplier)
        .withRotationalRate(-driverController.getRightX() * Parameters.kRotationSpeedAt12Volts.in(RadiansPerSecond))
)
          /*   () -> new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(Constants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * controllerDeadband * translationSpeedMultiplier)
                .withRotationalDeadband(Constants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * controllerDeadband * rotationSpeedMultiplier)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-driverController.getLeftY() * Constants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * translationSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * Constants.kTranslationSpeedAt12Volts.in(FeetPerSecond) * translationSpeedMultiplier)
                .withTargetDirection(Rotation2d.fromDegrees(NonZeroRad(-driverController.getRightX(), -driverController.getRightY())))
                .withTargetRateFeedforward(Constants.HeadingFF)
                .withHeadingPID(Constants.HeadingControlkP, Constants.HeadingControlkI, Constants.HeadingControlkD)
                // .withRotationa
                lRate(-driverController.getRightX() * 0.2 * Constants.kRotationSpeedAt12Volts.in(RadiansPerSecond) * rotationSpeedMultiplier)
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
    // Arm control
    driverController.leftTrigger().onTrue(Commands.runOnce(() -> arm.AutoGoToGround()));
    driverController.rightTrigger().onTrue(Commands.runOnce(() -> arm.AutoGoUp()));
    driverController.y().onTrue(Commands.runOnce(() -> arm.prepareForAlgae()));

    // Int/out control

    // D-pad down: intake coral
    driverController.povDown().onTrue(Commands.sequence(Commands.runOnce(() -> Intout.set(Parameters.one*0.1)), Commands.waitUntil(() -> !Intout.coralSwitch.get()), Commands.runOnce(() -> Intout.set(0))));
    // D-pad  up: outtake coral
    driverController.povUp().onTrue(Commands.sequence(Commands.runOnce(() -> Intout.set(-Parameters.one*0.1)), Commands.waitUntil(() -> Intout.coralSwitch.get()), Commands.runOnce(() -> Intout.set(0))));
    // D-pad left: intake algae
    driverController.povLeft().onTrue(Commands.sequence(Commands.runOnce(() -> Intout.set(Parameters.one*0.1)), Commands.waitUntil(() -> !Intout.algaeSwitch.get()), Commands.runOnce(() -> Intout.set(0))));
    // D-pad right: outtake algae
    driverController.povRight().onTrue(Commands.sequence(Commands.runOnce(() -> Intout.set(-Parameters.one*0.1)), Commands.waitUntil(() -> Intout.algaeSwitch.get()), Commands.runOnce(() -> Intout.set(0))));

    // Climb control
    driverController.b().onTrue(Commands.runOnce(() -> climb.climb()));
  }
  @Override
  public void autonomousInit() {
    // PGB: Put and Get Blue position: puts a preloaded coral in L1, then drives to the loader. Unifnished auto
    autos.A1("PGB1").cmd().schedule();
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
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

}
