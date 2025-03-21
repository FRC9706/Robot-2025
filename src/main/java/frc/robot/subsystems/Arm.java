// package frc.robot.subsystems;
// import frc.robot.ArmConstants;

// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ControlType;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

// public class Arm {
//     SparkMax max = new SparkMax(1, MotorType.kBrushless);
//     SparkMaxConfig config = new SparkMaxConfig();
//     SparkClosedLoopController pidController = max.getClosedLoopController();

//     public Arm() {
//         config.inverted(ArmConstants.kInverted).idleMode(IdleMode.kBrake);
//         config.encoder.positionConversionFactor(ArmConstants.kArmPositionConversionFactor).velocityConversionFactor(ArmConstants.kArmVelocityConversionFactor);
//         config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ArmConstants.kArmPgain, ArmConstants.kArmIgain, ArmConstants.kArmDgain);
//         max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     private Command goPosition(double position) {
//         return Commands.runOnce(() -> pidController.setReference(position, ControlType.kPosition));
//     }

//     public Command goToPosition1() {
//         return this.goPosition(ArmConstants.kArmPosition1);
//     }

//     public Command goToPosition2() {
//         return this.goPosition(ArmConstants.kArmPosition2);
//     }
// }

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;

public class Arm extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkClosedLoopController cloop = motor.getClosedLoopController();
    private static final double TOLERANCE = 0.001; // Acceptable error in rotations

    public Arm() {
        encoder.setPosition(0);
    }

    public void setTargetRotations(double pos) {
        cloop.setReference(pos, SparkMax.ControlType.kPosition);
    }

    public boolean isAtTarget(double target) {
        return Math.abs(encoder.getPosition() - target) < TOLERANCE;
    }

    public void stop() {
        motor.set(0);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }
}
