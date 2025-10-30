package frc.robot.subsystems;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DetectorConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Parameters;
import frc.robot.Robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

// previous failed fix counter: 14 different people - NAV FIXED IT LETS GODGHDSKJFDASJFGDSjfgdskfdskfs

public class Autos extends SubsystemBase {

    // private final AutoFactory autofact;
        private final AutoFactory autoFac;
        private final Swerve drivetrain = Robot.drivetrain;
        private final Arm arm = Arm.getInstance();
        public static Autos mInstance = null;
        public static Autos getInstance() {
            if (mInstance == null) {
                mInstance = new Autos();
            }
            return mInstance;
        }
    
        public Autos() {
            autoFac = new AutoFactory(() -> 
            drivetrain.getState().Pose, 
            drivetrain::resetPose, 
            drivetrain::followTrajectory, 
            false, 
            drivetrain);
        }
    
        public AutoRoutine scoreNdefend() {
            // give this routine a very helpful and descriptive name as you can see
            AutoRoutine autoRout = autoFac.newRoutine("andre");
    
            // Load trajectories
            AutoTrajectory andre = autoRout.trajectory("andre");
            AutoTrajectory andrep2 = autoRout.trajectory("andrep2");

            System.out.println("Loaded trajctories " + andre +  "and " + andrep2);
    
            autoRout.active().onTrue(
                Commands.sequence(
                    // reset odometry and run andre
                    andre.resetOdometry(),
                    Commands.runOnce(() -> System.out.println("RUNNING AUTO ROUTINE")),
                    andre.cmd()
                ) 
            );
    
            // after andre is done, go to the april tag which should (hopefully) infront of you
            andre.done().onTrue(Commands.sequence(
            Commands.run(() -> 
            
            // Go to the april tag & move directly into it
            drivetrain.goToAprilTag())
                .until(() -> LimelightHelpers.getTA(DetectorConstants.kLimelightName) >= 8.9),
            drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.5)).withTimeout(1),
            drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)).withTimeout(0),
    
            // Get arm ready to shoot
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> Arm.goToPos(Parameters.shootCor), drivetrain),
            Commands.waitUntil(() -> Math.abs(Arm.getPos() - Parameters.shootCor) < 0.5),
      
            // Shoot and retract arm
            Commands.waitSeconds(1),
            Commands.runOnce(() -> Intout.set(-Parameters.one), Intout.getInstance()),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> Intout.set(0), Intout.getInstance()),
            Commands.runOnce(() -> Arm.goToPos(Parameters.retract)),
    
            // Start andrep2 (the defending part)
            andrep2.cmd()
                )
            );
    
            // After andrep2 has finished, reset robot pos feild centric (press A)
            // andrep2.done().onTrue();
    
            return autoRout;
        }
    
    
    
    
        // public AutoRoutine A1(String name) {
        //     AutoRoutine routine = autofact.newRoutine(name);
        //     AutoTrajectory GoToReef = routine.trajectory(name, 0);
        //     AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        //     AutoTrajectory idklol = routine.trajectory(name, 2);
        //     AutoTrajectory GoBackToFeeder = routine.trajectory(name, 3);
        //     AutoTrajectory RunAway = routine.trajectory(name, 4);
        //     routine.active().onTrue(
        //         Commands.sequence(
        //             GoToReef.resetOdometry(),
        //             GoToReef.cmd(),
        //             Commands.runOnce(() -> arm.set(0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralOuttake()),
        //             Commands.waitSeconds(Parameters.AutoOuttakeWaitTime),
        //             GoToFeeder.resetOdometry(),
        //             GoToFeeder.cmd(),
        //             idklol.resetOdometry(),
        //             idklol.cmd(),
        //             Commands.runOnce(() -> arm.set(-0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralIntake()),
        //             GoBackToFeeder.resetOdometry(),
        //             GoBackToFeeder.cmd(),
        //             Commands.runOnce(() -> arm.set(1)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralOuttake()),
        //             RunAway.resetOdometry(),
        //             RunAway.cmd()
        //         )
        //     );
        //     return routine;
        // }
    
        // public AutoRoutine A2(String name) {
        //     AutoRoutine routine = autofact.newRoutine(name);
        //     AutoTrajectory GoToReef = routine.trajectory(name, 0);
        //     AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        //     AutoTrajectory idklol = routine.trajectory(name, 2);
        //     AutoTrajectory GoBackToFeeder = routine.trajectory(name, 3);
        //     AutoTrajectory RunAway = routine.trajectory(name, 4);
        //     routine.active().onTrue(
        //         Commands.sequence(
        //             GoToReef.resetOdometry(),
        //             GoToReef.cmd(),
        //             Commands.runOnce(() -> arm.set(0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralOuttake()),
        //             Commands.waitSeconds(Parameters.AutoOuttakeWaitTime),
        //             GoToFeeder.resetOdometry(),
        //             GoToFeeder.cmd(),
        //             idklol.resetOdometry(),
        //             idklol.cmd(),
        //             Commands.runOnce(() -> arm.set(-0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralIntake()),
        //             GoBackToFeeder.resetOdometry(),
        //             GoBackToFeeder.cmd(),
        //             Commands.runOnce(() -> arm.set(0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralOuttake()),
        //             RunAway.resetOdometry(),
        //             RunAway.cmd()
        //         )
        //     );
        //     return routine;
        // }
    
        // public AutoRoutine A3(String name) {
        //     AutoRoutine routine = autofact.newRoutine(name);
        //     AutoTrajectory GoToReef = routine.trajectory(name, 0);
        //     AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        //     AutoTrajectory GoBackToFeeder = routine.trajectory(name, 2);
        //     AutoTrajectory RunAway = routine.trajectory(name, 3);
        //     routine.active().onTrue(
        //         Commands.sequence(
        //             GoToReef.resetOdometry(),
        //             GoToReef.cmd(),
        //             Commands.runOnce(() -> arm.set(0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralOuttake()),
        //             Commands.waitSeconds(Parameters.AutoOuttakeWaitTime),
        //             GoToFeeder.resetOdometry(),
        //             GoToFeeder.cmd(),
        //             Commands.runOnce(() -> arm.set(-0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralIntake()),
        //             GoBackToFeeder.resetOdometry(),
        //             GoBackToFeeder.cmd(),
        //             Commands.runOnce(() -> arm.set(0.5)),
        //             Commands.waitSeconds(2),
        //             Commands.runOnce(() -> arm.set(0)),
        //             Commands.runOnce(() -> Intout.AutoCoralOuttake()),
        //             RunAway.resetOdometry(),
        //             RunAway.cmd()
        //         )
        //     );
        //     return routine;
        // }
        
        // public AutoRoutine GTFO() {
        //     AutoRoutine routine = autofact.newRoutine("GTFO");
        //     AutoTrajectory GTOF = routine.trajectory("GTOF", 0);
        //     routine.active().onTrue(
        //         Commands.sequence(
        //         GTOF.resetOdometry(),
        //         GTOF.cmd()
        //         )
        //     );
        //     return routine;
        // }
    
        public void Taxi() {
                Commands.sequence(
                Commands.print("yo bro ima move bro"),
                drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(1)).withTimeout(1),

                Commands.print("yo bro ima stop moving bro"),
                drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)).withTimeout(1)
              ).schedule();
    }
}    
    









