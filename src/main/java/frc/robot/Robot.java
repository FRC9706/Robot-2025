package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
  private final Arm arm = new Arm();
  private final double armTravelRots = 0.75;

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(0);

  public Robot() {

    // driverController.leftTrigger().onTrue(
    //   Commands.sequence(
    //     Commands.runOnce(() -> arm.setTargetRotations(-armTravelRots)),
    //     Commands.waitUntil(() -> arm.isAtTarget(-armTravelRots)),
    //     Commands.runOnce(arm::stop)
    //   ));

    // driverController.rightTrigger().onTrue(
    //   Commands.sequence(
    //     Commands.runOnce(() -> arm.setTargetRotations(armTravelRots)),
    //     Commands.waitUntil(() -> arm.isAtTarget(armTravelRots)),
    //     Commands.runOnce(arm::stop)
    //   ));

    driverController.leftTrigger().onTrue(Commands.runOnce(() -> arm.setTargetCentPos(50)));
    driverController.rightTrigger().onTrue(Commands.runOnce(() -> arm.setTargetCentPos(-50)));

  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

}
