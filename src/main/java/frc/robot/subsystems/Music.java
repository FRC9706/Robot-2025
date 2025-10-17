package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Parameters;

import java.util.ArrayList;
import java.util.List;

public class Music {

    private static Orchestra m_orchestra;
    private static final List<TalonFX> allMotors = new ArrayList<>();

    // Initialize all TalonFX motors from Parameters
    public static void initAllMotors() {
        allMotors.clear();

        // Add drivetrain TalonFXs
        allMotors.add(new TalonFX(Parameters.kFrontLeftDriveMotorId));
        allMotors.add(new TalonFX(Parameters.kFrontRightDriveMotorId));
        allMotors.add(new TalonFX(Parameters.kBackLeftDriveMotorId));
        allMotors.add(new TalonFX(Parameters.kBackRightDriveMotorId));

        allMotors.add(new TalonFX(Parameters.kFrontLeftSteerMotorId));
        allMotors.add(new TalonFX(Parameters.kFrontRightSteerMotorId));
        allMotors.add(new TalonFX(Parameters.kBackLeftSteerMotorId));
        allMotors.add(new TalonFX(Parameters.kBackRightSteerMotorId));
    }

    // Play a music file on all motors
    public static void playAll(String track) {
        if (m_orchestra == null) {
            m_orchestra = new Orchestra();
            initAllMotors();
            for (TalonFX motor : allMotors) {
                m_orchestra.addInstrument(motor);
            }
        }

        var status = m_orchestra.loadMusic(track);

        if (status.isOK()) {
            System.out.println("Loaded track: " + track);
            var playStatus = m_orchestra.play();
            if (!playStatus.isOK()) {
                System.out.println("Error: Could not start playing music.");
            }
        } else {
            System.out.println("Failed to load track: " + track);
        }
    }

    public static void stopAll() {
        if (m_orchestra != null) {
            m_orchestra.stop();
        }
    }

    public static void pauseAll() {
        if (m_orchestra != null) {
            m_orchestra.pause();
        }
    }

    public static void unpauseAll() {
        if (m_orchestra != null) {
            m_orchestra.play();
        }
    }
}
