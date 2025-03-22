package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intout extends SubsystemBase {
    private final static SparkMax intout = new SparkMax(Constants.kIntakeMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    public Intout() {
        intout.set(0);
    }

    public static void outtake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(1)), Commands.waitSeconds(Constants.kShootDuration), Commands.runOnce(() -> intout.set(0)));
    }

    public static void intake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(-1)), Commands.waitSeconds(Constants.kIntakeDuration), Commands.runOnce(() -> intout.set(0)));
    }
}
