package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
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
    public static SparkClosedLoopController CLcontroller = motor.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder = motor.getEncoder();
    // static public final SparkClosedLoopController cloop = motor.getClosedLoopController();
    private boolean algaePrepared = false;

    public Arm() {
        encoder
        .setPosition(-6.786);

        config
        .inverted(false);

        config
        .idleMode(IdleMode.kBrake);

        config
        .smartCurrentLimit(10);

        config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(Parameters.minOut, Parameters.mxOut)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(Parameters.velFF, ClosedLoopSlot.kSlot1)
        .outputRange(Parameters.minOut, Parameters.mxOut, ClosedLoopSlot.kSlot1);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public boolean isOverLim = false;
    public boolean isMoving = false;

    public static void goToPos(double targPos) {
        CLcontroller.setReference(targPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void cMove(double sped) {
        if (!isOverLim) {
            motor.set(sped);
            isMoving = true;
        } else {
            System.out.println("STOP MOVING THE ARM ANDRE");
            motor.stopMotor();
            isMoving = false;
        }
    }

    public static double getPos() {
        double position = motor.getEncoder().getPosition();
            return position;
    }
    
    

    // public void setTargetPos(double pos) {
    //     cloop.setReference(pos, SparkMax.ControlType.kPosition);
    // }

    public void set(double sped) {
        motor.set(sped);
        System.out.println("I just set ur arm to the speed: " + sped);
    }

    // public void prepareForAlgae() {
    //     if (algaePrepared){
    //     config.idleMode(IdleMode.kCoast);
    //     motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //     algaePrepared = false;
    // } else {
    //     setTargetPos(Parameters.kArmAlgae);
    //     config.idleMode(IdleMode.kBrake);
    //     motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //     algaePrepared = true;
    // }
    // }

    // public void AutoGoToGround() {
    //     setTargetPos(Parameters.kArmPos1);
    // }

    // public void AutoGoUp() {
    //     setTargetPos(Parameters.kArmPos2);
    // }

    @Override
    public void periodic() {
        if ((Math.abs(getPos())) < 10) {
            if (isMoving) {
            motor.stopMotor();
            isMoving = false;
            }
            isOverLim = true;
        } else {
            isOverLim = false;
        }
    }
}