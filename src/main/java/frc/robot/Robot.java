package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import static edu.wpi.first.units.Units.FeetPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.spark.ClosedLoopSlot;
//import com.fasterxml.jackson.core.base.ParserMinimalBase;
import com.revrobotics.spark.SparkBase.ControlType;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intout;
import frc.robot.subsystems.Music;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Autos;

public class Robot extends TimedRobot {

  // initialize subsystems
  public final static Swerve drivetrain = Parameters.createDrivetrain();
  private final Arm arm = Arm.getInstance();
  private final Intout intout = new Intout();
  private final Climb climb = Climb.getInstance();
  private final Autos autos = Autos.getInstance();
  private static final String kB1 = "andreB";
  private static final String kB2 = "andreBRight";
  private static final String kB3 = "andreBLeft";
  private static final String kR1 = "andre";
  private static final String kR2 = "andreRight";
  private static final String kR3 = "andreLeft";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(1);

  public Robot() {

    m_chooser.addOption("BlueMiddle", kB1);
    m_chooser.addOption("BlueRight", kB2);
    m_chooser.addOption("BlueLeft", kB3);
    m_chooser.addOption("RedMiddle", kR1);
    m_chooser.addOption("RedRight", kR2);
    m_chooser.addOption("RedLeft", kR3);
    SmartDashboard.putData("Pick an auto, any auto:", m_chooser);

    // Configure DogLog
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureConsole(true)
    );
    

    // Teleop Speed Multipliers. Percentages of the max speed. 
    double translationSpeedMultiplier = 1;
    double controllerDeadband = 0;

    new Rotation2d();
    // Drive command
    final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(Parameters.kTranslationSpeedAt12Volts.in(FeetPerSecond) * controllerDeadband * translationSpeedMultiplier)
      .withDriveRequestType(DriveRequestType.Velocity);
    snapDrive.HeadingController = new PhoenixPIDController(Parameters.HeadingControlkP, Parameters.HeadingControlkI, Parameters.HeadingControlkD);
    snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the default command for the drivetrain to be the teleop drive command.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> new SwerveRequest.FieldCentric()
                .withDeadband(Parameters.kTranslationSpeedAt12Volts.in(MetersPerSecond) * controllerDeadband * translationSpeedMultiplier)
                .withRotationalDeadband(Parameters.kRotationSpeedAt12Volts.in(RadiansPerSecond) * controllerDeadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-driverController.getLeftY() * Parameters.kTranslationSpeedAt12Volts.in(MetersPerSecond) * translationSpeedMultiplier)
                .withVelocityY(-driverController.getLeftX() * Parameters.kTranslationSpeedAt12Volts.in(MetersPerSecond) * translationSpeedMultiplier)
                .withRotationalRate(-driverController.getRightX() *0.8 * Parameters.kRotationSpeedAt12Volts.in(RadiansPerSecond))
        )
    );
      

    // =============== Controller Bottons ===============

    // driverController.x().onTrue(
    //   Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.RobotCentric()
    //   .withRotationalRate(0)
    //   .withVelocityX(0))
    //   )
    // );

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

    // driverController.x().whileTrue(
    //   Commands.run(() -> {
    //       drivetrain.goToAprilTag();
    //   }, drivetrain)
    //   //  .until(() -> LimelightHelpers.getTA(DetectorConstants.kLimelightName) >= 2)
    // );
    
    driverController.y().onTrue(Commands.runOnce(() -> Climb.goToRotPlusOne()));

  // Arm control v2
  
  driverController.povRight().onTrue(Commands.runOnce(() -> Arm.goToPos(Parameters.retract)));
  driverController.povUp().onTrue(Commands.runOnce(() -> Arm.goToPos(Parameters.shootCor)));
  driverController.povDown().onTrue(Commands.runOnce(() -> Arm.goToPos(Parameters.grabCor)));
  // driverController.povLeft().onTrue(Commands.runOnce(() -> Arm.goToPos(Parameters.grabCor)));


  // Arm control
  driverController.leftTrigger().whileTrue(Commands.runOnce(() -> arm.cMove(0.5)));
  driverController.leftTrigger().onFalse(Commands.runOnce(() -> arm.set(0)));
  driverController.rightTrigger().onTrue(Commands.runOnce(() -> arm.set(-0.75)));
  driverController.rightTrigger().onFalse(Commands.runOnce(() -> arm.set(0)));
  

  // Int/out control

  // left bumper: intake coral
  //  driverController.leftBumper().whileTrue(
  //   Commands.sequence(
  //     Commands.run(() -> Intout.set(Parameters.one))  // Run intake continuously
  //    Commands.waitUntil(() -> !Intout.coralSwitch.get())  // Stop when the limit switch is triggered
  // )).onFalse(
  //   Commands.runOnce(() -> Intout.set(0))  // Stop motor when button is released
  // );

  // // right bumper: outtake coral
  // driverController.rightBumper().whileTrue(
  //   Commands.sequence(
  //     Commands.run(() -> Intout.set(-Parameters.one))  // Run outtak`e continuously
  //     Commands.waitUntil(() -> Intout.coralSwitch.get())  // Stop when the limit switch is triggered
  //   )).onFalse(
  //   Commands.runOnce(() -> Intout.set(0))  // Stop motor when button is released
  // );
    
    // left bumber: intake algae
    driverController.leftBumper().whileTrue(
      Commands.sequence(
        Commands.runOnce(() -> Intout.set(Parameters.one))
       // Commands.waitUntil(() -> Intout.algaeSwitch.get()),
      )).onFalse(
        Commands.runOnce(() -> Intout.set(0))
      );

    // right bumper: outtake algae
    driverController.rightBumper().onTrue(
      Commands.sequence(
      Commands.runOnce(() -> Intout.set(-Parameters.one))
     // Commands.waitUntil(() -> !Intout.algaeSwitch.get()),
      )).onFalse(
        Commands.runOnce(() -> Intout.set(0))
      );


  }
  AutoFactory autoFac = new AutoFactory(() -> 
            drivetrain.getState().Pose, 
            drivetrain::resetPose, 
            drivetrain::followTrajectory, 
            false, 
            drivetrain);
                AutoRoutine autoRout = autoFac.newRoutine("andre");
  AutoTrajectory andreB =autoRout.trajectory("andreB");  
  AutoTrajectory andreBP2 =autoRout.trajectory("andreBP2");  
  AutoTrajectory andreBRight =autoRout.trajectory("andreBRight");  
  AutoTrajectory andreBRightP2 =autoRout.trajectory("andreBRightP2");  
  AutoTrajectory andreBLeft =autoRout.trajectory("andreBLeft");  
  AutoTrajectory andreBLeftP2 =autoRout.trajectory("andreBLeftP2");  
  AutoTrajectory andre =autoRout.trajectory("andre");  
  AutoTrajectory andreP2 =autoRout.trajectory("andreP2");  
  AutoTrajectory andreRight =autoRout.trajectory("andreRight");  
  AutoTrajectory andreRightP2 =autoRout.trajectory("andreRightP2");  
  AutoTrajectory andreLeft =autoRout.trajectory("andreLeft");  
  AutoTrajectory andreLeftP2 =autoRout.trajectory("andreLeftP2");  
  @Override
  public void autonomousInit() {
    
    // Optional: Select an autonomous routine based on a chooser, if you decide to use it
    m_autoSelected = m_chooser.getSelected();
    switch (m_autoSelected) {
        case kB1: autos.scoreNdefend(autoRout, andreB, andreBP2).cmd().schedule(); break;
        case kB2: autos.scoreNdefend(autoRout, andreBRight, andreBRightP2).cmd().schedule(); break;
        case kB3: autos.scoreNdefend(autoRout, andreBLeft, andreBLeftP2).cmd().schedule(); break;
        case kR1: autos.scoreNdefend(autoRout, andre, andreP2).cmd().schedule(); break;
        case kR2: autos.scoreNdefend(autoRout, andreRight, andreRightP2).cmd().schedule(); break;
        case kR3: autos.scoreNdefend(autoRout, andreLeft, andreLeftP2).cmd().schedule(); break;
    }

    // Ensure the drivetrain is reset to a neutral state to prevent any conflicts
    
    // launch the auto
    // autos.scoreNdefend().cmd().schedule();
    // This code works on magic dont touch
  //  Commands.sequence(
  //     Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero), drivetrain),
  //     Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.k180deg), drivetrain),
  //     drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.5)).withTimeout(5)

      // Commands.run(() -> drivetrain.goToAprilTag(), drivetrain)
      //     .until(() -> LimelightHelpers.getTA(DetectorConstants.kLimelightName) >= 8.9),
      //     drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.5)).withTimeout(1),
      //     drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)).withTimeout(0),


      // Commands.waitSeconds(0.5),
      // Commands.runOnce(() -> Arm.goToPos(Parameters.shootCor), drivetrain),
      // Commands.waitUntil(() -> Math.abs(Arm.getPos() - Parameters.shootCor) < 0.5),

      // Commands.waitSeconds(1),
      // Commands.runOnce(() -> Intout.set(-Parameters.one), Intout.getInstance()),
      // Commands.waitSeconds(0.5),
      // Commands.runOnce(() -> Intout.set(0), Intout.getInstance()),
      // Commands.runOnce(() -> Arm.goToPos(Parameters.retract))

  // ).schedule();
}

    @Override
  public void teleopInit() {

    drivetrain.applyRequestOnce(
        () -> new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
    ).schedule();

    // double targetPosition = SmartDashboard.getNumber("Target Position", 0);
    // Arm.CLcontroller.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

}

      @Override
  public void robotInit() { 
    SignalLogger.stop(); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

}
