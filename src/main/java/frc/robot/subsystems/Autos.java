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

    public Autos() {
        autofact = new AutoFactory(() -> drivetrain.getState().Pose, drivetrain::resetPose, drivetrain::followTrajectory, true, drivetrain);
    }

    public AutoRoutine PGB1() {
        AutoRoutine routine = autofact.newRoutine("test");
        AutoTrajectory reefTraj = routine.trajectory("PutAndGetBlu1", 0);
        routine.active().onTrue(
            Commands.sequence(
                reefTraj.resetOdometry(),
                reefTraj.cmd()
            )
        );
        return routine;
    }
    
}
