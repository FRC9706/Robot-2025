package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveConstants;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
    private static final TalonFX climbMotor = new TalonFX(SwerveConstants.kClimbMotorID);
    private boolean climberPosition = false;
    private boolean isRunning = false;
    
    public Climb() {
        climbMotor.set(0);
    }

    public void climb() {
        if (climberPosition) {
            if (isRunning) {
                climbMotor.set(0);
                isRunning = false;
                climberPosition = false;
            } else {
                climbMotor.set(-0.1);
                isRunning = true;
            }
        } else {
            if (isRunning) {
                climbMotor.set(0);
                isRunning = false;
                climberPosition = true;
            } else {
                climbMotor.set(0.1);
                isRunning = true;
            }
        }
    }
}
