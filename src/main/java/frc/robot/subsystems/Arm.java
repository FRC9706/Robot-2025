package frc.robot.subsystems;
import frc.robot.ArmConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class Arm {
    SparkMax max = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    SparkClosedLoopController cont = max.getClosedLoopController();

    public Arm() {
        config.inverted(ArmConstants.kInverted).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(ArmConstants.kArmPositionConversionFactor).velocityConversionFactor(ArmConstants.kArmVelocityConversionFactor);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ArmConstants.kArmPgain)
        .i(ArmConstants.kArmIgain)
        .d(ArmConstants.kArmDgain)
        .outputRange(-0.1, 0.1)
        .p(ArmConstants.kArmPgain, ClosedLoopSlot.kSlot1)
        .i(ArmConstants.kArmIgain, ClosedLoopSlot.kSlot1)
        .d(ArmConstants.kArmDgain, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput, ClosedLoopSlot.kSlot1);
        config.signals.primaryEncoderPositionPeriodMs(1);
        // set all the configs to the motor
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize dashboard values? Maybe they are needed, not sure yet.
        
        // SmartDashboard.setDefaultNumber("Target Position", 0);
        // SmartDashboard.setDefaultNumber("Target Velocity", 0);
        // SmartDashboard.setDefaultBoolean("Control Mode", false);
        // SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    public double getPos(){
        double currentPos = max.getEncoder().getPosition();
        return currentPos;
    }
    

    public void goPosition(double position) {
        // Set the desired position
        double currPos = max.getEncoder().getPosition();
        cont.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    
        // Wait until the position is reached with a small delay
        while (Math.abs(currPos - position) > 0.001) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            currPos = max.getEncoder().getPosition();  // Update current position
        }
    
        // Once the loop exits, the arm is close enough to the desired position.
        // Stop the motor
        cont.setReference(0, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
    }
    
    

    public void stopMotor() {
        cont.setReference(0, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
    }
    

    public void goToPosition1() {
        this.goPosition(ArmConstants.kArmPosition1);
    }

    public void goToPosition2() {
        this.goPosition(ArmConstants.kArmPosition2);
    }
}
