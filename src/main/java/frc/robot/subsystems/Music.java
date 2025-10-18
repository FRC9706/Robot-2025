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

        // Add drivetrain TalonFX motors
        allMotors.add(new TalonFX(Parameters.kFrontLeftDriveMotorId));
        allMotors.add(new TalonFX(Parameters.kFrontRightDriveMotorId));
        allMotors.add(new TalonFX(Parameters.kBackLeftDriveMotorId));
        allMotors.add(new TalonFX(Parameters.kBackRightDriveMotorId));

        // Add steering TalonFX motors
        allMotors.add(new TalonFX(Parameters.kFrontLeftSteerMotorId));
        allMotors.add(new TalonFX(Parameters.kFrontRightSteerMotorId));
        allMotors.add(new TalonFX(Parameters.kBackLeftSteerMotorId));
        allMotors.add(new TalonFX(Parameters.kBackRightSteerMotorId));
    }

    // Load the music file onto all motors (call this once before play)
    public static void loadAll(String track) {
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
        } else {
            System.out.println("Failed to load music: " + status.toString());
        }
    }


    // Start or resume music playback (assumes music already loaded)
    public static void playAll() {
        if (m_orchestra != null) {
            var playStatus = m_orchestra.play();
            if (playStatus.isOK()) {
                System.out.println("Playing music!");
            } else {
                System.out.println("Error: Could not start playing music.");
            }
        }
    }

    // Stop music playback and release control of the motors
    public static void stopAll() {
        if (m_orchestra != null) {
            m_orchestra.stop();
            System.out.println("Music stopped.");
        }
    }

    // Pause music playback
    public static void pauseAll() {
        if (m_orchestra != null) {
            m_orchestra.pause();
            System.out.println("Music paused.");
        }
    }

    // Unpause/resume music playback
    public static void unpauseAll() {
        if (m_orchestra != null) {
            var playStatus = m_orchestra.play();
            if (playStatus.isOK()) {
                System.out.println("Music resumed.");
            } else {
                System.out.println("Error: Could not resume music.");
            }
        }
    }
}
