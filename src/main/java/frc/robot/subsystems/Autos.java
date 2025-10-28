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
import frc.robot.Parameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

// previous failed fix counter: 14 different people

public class Autos extends SubsystemBase {

    // private final AutoFactory autofact;
    private final Swerve drivetrain = Parameters.createDrivetrain();
    private final Arm arm = Arm.getInstance();
    public static Autos mInstance = null;
    public static Autos getInstance() {
        if (mInstance == null) {
            mInstance = new Autos();
        }
        return mInstance;
    }

    public Autos() {
        AutoFactory Autofac = new AutoFactory(() -> 
        drivetrain.getState().Pose, 
        drivetrain::resetPose, 
        drivetrain::followTrajectory, 
        false, 
        drivetrain);
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
                drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric()
                .withVelocityX(1))
                .withTimeout(1),
                drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric()
                .withVelocityX(0)).withTimeout(1),
                Commands.print("yo bro ima stop moving bro")
              ).schedule();

    }
}    
    









