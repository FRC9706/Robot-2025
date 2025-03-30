package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Parameters;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intout extends SubsystemBase {
    private final static SparkMax intout = new SparkMax(Parameters.kIntakeMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final static DigitalInput coralSwitch = new DigitalInput(Parameters.kCoralLimitSwitchID);
    private final static DigitalInput algaeSwitch = new DigitalInput(Parameters.kAlgaeLimitSwitchID);

    public Intout() {
        intout.set(0);
    }

    public static void set(double speed) {
        intout.set(speed);
    }

    public static void outtake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(Parameters.one)), Commands.waitSeconds(Parameters.kShootDuration), Commands.runOnce(() -> intout.set(0)));
    }

    public static void intake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(-Parameters.one)), Commands.waitSeconds(Parameters.kIntakeDuration), Commands.runOnce(() -> intout.set(0)));
    }

    public static void AutoCoralOuttake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(Parameters.one)), Commands.waitUntil(() -> !coralSwitch.get()), Commands.runOnce(() -> intout.set(0)));
    }

    public static void AutoCoralIntake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(-Parameters.one)), Commands.waitUntil(() -> coralSwitch.get()), Commands.runOnce(() -> intout.set(0)));
    }

    public static void AutoAlgaeIntake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(-Parameters.one)), Commands.waitUntil(() -> algaeSwitch.get()), Commands.runOnce(() -> intout.set(0)));
    }

    public static void AutoAlgaeOuttake() {
        Commands.sequence(Commands.runOnce(() -> intout.set(Parameters.one)), Commands.waitUntil(() -> !algaeSwitch.get()), Commands.runOnce(() -> intout.set(0)));
    }
}
