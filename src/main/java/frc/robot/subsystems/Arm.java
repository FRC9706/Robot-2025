package frc.robot.subsystems;
import frc.robot.ArmConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Arm {
    SparkMax max = new SparkMax(1, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    SparkClosedLoopController pidController = max.getClosedLoopController();

    public Arm() {
        config.inverted(ArmConstants.kInverted).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(ArmConstants.kArmPositionConversionFactor).velocityConversionFactor(ArmConstants.kArmVelocityConversionFactor);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ArmConstants.kArmPgain, ArmConstants.kArmIgain, ArmConstants.kArmDgain);
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private Command goPosition(double position) {
        return Commands.runOnce(() -> pidController.setReference(position, ControlType.kPosition));
    }

    public Command goToPosition1() {
        return this.goPosition(ArmConstants.kArmPosition1);
    }

    public Command goToPosition2() {
        return this.goPosition(ArmConstants.kArmPosition2);
    }
}
