// package frc.robot.subsystems;

// import com.ctre.phoenix6.Orchestra;
// import com.ctre.phoenix6.hardware.*;


// public class Music {

//     private static TalonFX motor;
//     private static Orchestra m_orchestra;

//     // Method to create a TalonFX motor
//     public static TalonFX musicMotor(int motorid) {
//         motor = new TalonFX(motorid);  // Instantiate the TalonFX motor with the motor ID
//         return motor;  // Return the created motor instance
//     }

//     // A method that creates the orchestra, adds the instrument (motor), and plays music
//     public static void playMusic(int motorid, String track) {
//         Orchestra m_orchestra = new Orchestra();  // Instantiate the Orchestra class
//         m_orchestra.addInstrument(musicMotor(motorid));  // Add the motor to the orchestra as an instrument

//         // Load the music track
//         var status = m_orchestra.loadMusic(track);

//         if (!status.isOK()) {
//             System.out.println("Succesfully loaded the following track: " + track );
//          }


//          // play the music track
//          m_orchestra.play();

//          if (!m_orchestra.play().isOK()) {
//             System.out.println("I got no idea what went wrong vro");
//          }

//     }


//     // Pauses the music (idk how to unpause lol)
//     public void pauseMusic() {
//         if (m_orchestra != null) {
//             m_orchestra.pause();
//         } else {
//             System.out.println("It appears that m_orchestra is null (idk what ur doing vro)");
//         }
//     }

//     // I hope this works, it prob should
//     public void unpauseMusic() {
//         if (m_orchestra != null) {
//             m_orchestra.play();
//         } else {
//             System.out.println("It appears that m_orchestra is null (idk what ur doing vro)");
//         }
//     }

//     // Pretty much should kill the music, you would have to run playMusic() again I think

//     public void endMusic() {
//         if (m_orchestra != null) {
//             m_orchestra.stop();
//         } else {
//             System.out.println("It appears that m_orchestra is null vro HOW r u dum");
//         }
//     }


// }