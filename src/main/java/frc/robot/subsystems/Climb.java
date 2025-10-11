package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;

public class Climb extends SubsystemBase {
    public static final TalonFX climbMotor = new TalonFX(Parameters.kClimbMotorID);
    public static final TalonFX climbMotor2 = new TalonFX(Parameters.kClimbMotorID2);

    private boolean isRunning = false;
    public static Climb mInstance = null;
    public static Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }
    


    public Climb() {
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // other motor configs
        talonFXConfigs.CurrentLimits = Parameters.climbCurConfigs;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        var talonFXConfigs2 = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs2 = talonFXConfigs.Slot0;
        slot0Configs2.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs2.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs2.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs2.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs2.kI = 0; // no output for integrated error
        slot0Configs2.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        
        // set Motion Magic settings
        var motionMagicConfigs2 = talonFXConfigs.MotionMagic;
        motionMagicConfigs2.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs2.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs2.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // other motor configs
        talonFXConfigs2.CurrentLimits = Parameters.climbCurConfigs;
        talonFXConfigs2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs2.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        climbMotor.getConfigurator().apply(talonFXConfigs);

        climbMotor2.getConfigurator().apply(talonFXConfigs2);
        climbMotor2.setControl(new Follower(Parameters.kClimbMotorID, false)); // true was ben 10 iq


        climbMotor.setPosition(0);
        climbMotor2.setPosition(0);
    }

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public void climb() {
            if (isRunning) {
                climbMotor.set(0);
            } else {
                climbMotor.set(1);
            }
            isRunning = !isRunning; // even better
    }

    public double armDegsToMotorDegs(double input) {
        return input*5.091;
    }

    public void goToRot(double rot) {
        climbMotor.setControl(m_request.withPosition(rot));

    }
}
