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

    public Arm() {
        encoder.setPosition(0);
    }

    public void setTargetCentPos(double pos) {
        cloop.setReference(pos, SparkMax.ControlType.kPosition);
    }
}