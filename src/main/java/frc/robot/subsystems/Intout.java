package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Parameters;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class Intout extends SubsystemBase {
    private final static SparkMax intout = new SparkMax(Parameters.kIntakeMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    public static SparkClosedLoopController CLcontroller = intout.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder = intout.getEncoder();

    public final static DigitalInput coralSwitch = new DigitalInput(Parameters.kCoralLimitSwitchID);
    public final static DigitalInput algaeSwitch = new DigitalInput(Parameters.kAlgaeLimitSwitchID);
    public static Intout mInstance = null;
    public static Intout getInstance() {
        if (mInstance == null) {
            mInstance = new Intout();
        }
        return mInstance;
    }

    public Intout() {
        encoder
        .setPosition(0);

        config
        .inverted(false);

        config
        .idleMode(IdleMode.kCoast);

        config
        .smartCurrentLimit(60);

        config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(1, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(Parameters.velFF, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        intout.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public static void set(double targetVel) {
        // CLcontroller.setReference(targetVel, ControlType.kVelocity);
        intout.set(targetVel);
    }

    public static void stopMotor() {
        intout.stopMotor();
    }

    public static void outtake() {
        Commands.sequence(
            Commands.runOnce(() -> intout.set(Parameters.one)), 
            Commands.waitSeconds(Parameters.kShootDuration), 
            Commands.runOnce(() -> intout.set(0))
        );
    }

    public static void intake() {
        Commands.sequence(
            Commands.runOnce(() -> intout.set(-Parameters.one)), 
            Commands.waitSeconds(Parameters.kIntakeDuration), 
            Commands.runOnce(() -> intout.set(0))
        );
    }

    public static void AutoCoralOuttake() {
        Commands.sequence(
            Commands.runOnce(() -> intout.set(Parameters.one)), 
            Commands.waitUntil(() -> !coralSwitch.get()), 
            Commands.runOnce(() -> intout.set(0))
        );
    }

    public static void AutoCoralIntake() {
        Commands.sequence(
            Commands.runOnce(() -> intout.set(-Parameters.one)), 
            Commands.waitUntil(() -> coralSwitch.get()), 
            Commands.runOnce(() -> intout.set(0))
        );
    }

    public static void AutoAlgaeIntake() {
        Commands.sequence(
            Commands.runOnce(() -> intout.set(-Parameters.one)), 
            Commands.waitUntil(() -> algaeSwitch.get()), 
            Commands.runOnce(() -> intout.set(0))
        );
    }

    public static void AutoAlgaeOuttake() {
        Commands.sequence(
            Commands.runOnce(() -> intout.set(Parameters.one)), 
            Commands.waitUntil(() -> !algaeSwitch.get()), 
            Commands.runOnce(() -> intout.set(0))
        );
    }
}
