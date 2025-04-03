package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Arm extends SubsystemBase {
    public static Arm mInstance = null;
    public static Arm getInstance(){
        if(mInstance==null){
            mInstance = new Arm();
        }
        return mInstance;
    }
    static private final SparkMax motor = new SparkMax(Parameters.kArmMotorID, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkClosedLoopController cloop = motor.getClosedLoopController();
    private boolean algaePrepared = false;

    public Arm() {
        encoder.setPosition(0);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(10);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setTargetPos(double pos) {
        cloop.setReference(pos, SparkMax.ControlType.kPosition);
    }

    public void prepareForAlgae() {
        if (algaePrepared){
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        algaePrepared = false;
    } else {
        setTargetPos(Parameters.kArmAlgae);
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        algaePrepared = true;
    }
    }

    public void AutoGoToGround() {
        setTargetPos(Parameters.kArmPos1);
    }

    public void AutoGoUp() {
        setTargetPos(Parameters.kArmPos2);
    }
}