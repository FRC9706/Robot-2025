package frc.robot.subsystems;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Autos extends SubsystemBase {

    private final AutoFactory autofact;
    private final Swerve drivetrain = Constants.createDrivetrain();
    private final Arm arm = new Arm();

    public Autos() {
        autofact = new AutoFactory(() -> drivetrain.getState().Pose, drivetrain::resetPose, drivetrain::followTrajectory, true, drivetrain);
    }

    public AutoRoutine A1(String name) {
        AutoRoutine routine = autofact.newRoutine(name);
        AutoTrajectory GoToReef = routine.trajectory(name, 0);
        AutoTrajectory GoToFeeder = routine.trajectory(name, 1);
        routine.active().onTrue(
            Commands.sequence(
                GoToReef.resetOdometry(),
                GoToReef.cmd(),
                Commands.runOnce(() -> arm.AutoGoUp()),
                Commands.runOnce(() -> Intout.AutoOuttake()),
                Commands.waitSeconds(Constants.AutoOuttakeWaitTime),
                GoToFeeder.resetOdometry(),
                GoToFeeder.cmd(),
                Commands.runOnce(() -> arm.AutoGoToGround()),
                Commands.runOnce(() -> Intout.AutoIntake())
            )
        );
        return routine;
    }
    
}
