package frc.robot.autos;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class Auto {
    private DrivetrainSubsystem drivetrain;
    private AutoFactory autoFactory;
    private AutoRoutine routine;
    private final Choreo.TrajectoryCache cache;

    public Auto(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        // Initialize trajectory cache - preloads trajectories at robot startup
        // This prevents delays when loading trajectories during auto
        cache = new Choreo.TrajectoryCache();

        // Preload all trajectories you'll use in autonomous
        cache.loadTrajectory("TestPath");
        // Add more as needed:
        // cache.loadTrajectory("Path2");
        // cache.loadTrajectory("Path3");

        this.autoFactory = new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::stageTrajectory,
                true, // Trajectories are relative to starting pose
                drivetrain);

    }

    public AutoRoutine testRoutine() {

        AutoRoutine routine = autoFactory.newRoutine("Test Auto");

        // The trajectory method will automatically use the cache if the trajectory was
        // preloaded
        // Otherwise it loads from disk (which is slower)
        AutoTrajectory driveToMiddle = routine.trajectory("TestPath");

        // When the routine becomes active, reset odometry then follow the trajectory
        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started the routine!"),
                        driveToMiddle.resetOdometry(), // Reset pose to trajectory start
                        driveToMiddle.cmd() // Follow the trajectory
                ));

        return routine;
    }

    public Command setAutoRoutine() {
        // return Commands.defer(routine.cmd()::init,routine.cmd()::execute,
        // routine.cmd()::end,(()->routine.cmd()::isFinished)), Set.of(drivetrain));
        // AutoRoutine routine = autoFactory.newRoutine("Test Auto");
        // AutoTrajectory driveToMiddle = routine.trajectory("TestPath");

        // routine.active().onTrue(
        // Commands.sequence(

        // driveToMiddle.resetOdometry(),
        // driveToMiddle.cmd()));
        return testRoutine().cmd();
    }
}
