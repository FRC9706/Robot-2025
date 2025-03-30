package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
    private static final TalonFX climbMotor = new TalonFX(Parameters.kClimbMotorID);
    private boolean isRunning = false;
    
    public Climb() {
        climbMotor.set(0);
    }

    public void climb() {
        if (isRunning) {
            climbMotor.set(0);
            isRunning = false;
        } else {
            climbMotor.set(-0.1);
            isRunning = true;
        }
    }
}
