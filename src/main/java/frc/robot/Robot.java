package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
  private final Arm arm = new Arm();

  // Driver Controller
  private CommandXboxController driverController = new CommandXboxController(0);

  public Robot() {

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
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

}
