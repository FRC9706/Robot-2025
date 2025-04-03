package frc.robot.subsystems;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;

public class Autos extends SubsystemBase {

    private final AutoFactory autofact;
    private final Swerve drivetrain = Parameters.createDrivetrain();
    private final Arm arm = Arm.getInstance();

    public Autos() {
        autofact = new AutoFactory(() -> drivetrain.getState().Pose, drivetrain::resetPose, drivetrain::followTrajectory, false, drivetrain);
    }

    public AutoRoutine A1(String name) {
        AutoRoutine routine = autofact.newRoutine(name);
        AutoTrajectory GoToReef = routine.trajectory(name, 0);
        AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        AutoTrajectory idklol = routine.trajectory(name, 2);
        AutoTrajectory GoBackToFeeder = routine.trajectory(name, 3);
        AutoTrajectory RunAway = routine.trajectory(name, 4);
        routine.active().onTrue(
            Commands.sequence(
                GoToReef.resetOdometry(),
                GoToReef.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoCoralOuttake()),
                Commands.waitSeconds(Parameters.AutoOuttakeWaitTime),
                GoToFeeder.resetOdometry(),
                GoToFeeder.cmd(),
                idklol.resetOdometry(),
                idklol.cmd(),
                Commands.runOnce(() -> arm.AutoGoToGround()),
                Commands.runOnce(() -> Intout.AutoCoralIntake()),
                GoBackToFeeder.resetOdometry(),
                GoBackToFeeder.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoCoralOuttake()),
                RunAway.resetOdometry(),
                RunAway.cmd()
            )
        );
        return routine;
    }

    public AutoRoutine A2(String name) {
        AutoRoutine routine = autofact.newRoutine(name);
        AutoTrajectory GoToReef = routine.trajectory(name, 0);
        AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        AutoTrajectory idklol = routine.trajectory(name, 2);
        AutoTrajectory GoBackToFeeder = routine.trajectory(name, 3);
        AutoTrajectory RunAway = routine.trajectory(name, 4);
        routine.active().onTrue(
            Commands.sequence(
                GoToReef.resetOdometry(),
                GoToReef.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoCoralOuttake()),
                Commands.waitSeconds(Parameters.AutoOuttakeWaitTime),
                GoToFeeder.resetOdometry(),
                GoToFeeder.cmd(),
                idklol.resetOdometry(),
                idklol.cmd(),
                Commands.runOnce(() -> arm.AutoGoToGround()),
                Commands.runOnce(() -> Intout.AutoCoralIntake()),
                GoBackToFeeder.resetOdometry(),
                GoBackToFeeder.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoCoralOuttake()),
                RunAway.resetOdometry(),
                RunAway.cmd()
            )
        );
        return routine;
    }

    public AutoRoutine A3(String name) {
        AutoRoutine routine = autofact.newRoutine(name);
        AutoTrajectory GoToReef = routine.trajectory(name, 0);
        AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        AutoTrajectory GoBackToFeeder = routine.trajectory(name, 2);
        AutoTrajectory RunAway = routine.trajectory(name, 3);
        routine.active().onTrue(
            Commands.sequence(
                GoToReef.resetOdometry(),
                GoToReef.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoCoralOuttake()),
                Commands.waitSeconds(Parameters.AutoOuttakeWaitTime),
                GoToFeeder.resetOdometry(),
                GoToFeeder.cmd(),
                Commands.runOnce(() -> arm.AutoGoToGround()),
                Commands.runOnce(() -> Intout.AutoCoralIntake()),
                GoBackToFeeder.resetOdometry(),
                GoBackToFeeder.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoCoralOuttake()),
                RunAway.resetOdometry(),
                RunAway.cmd()
            )
        );
        return routine;
    }
    
}
