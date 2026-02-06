package frc.robot.autos;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class Auto {
    private DrivetrainSubsystem drivetrain;
    private AutoFactory autoFactory;
    private AutoRoutine routine;

    public Auto(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.autoFactory = new AutoFactory(

                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain

        );

    }

    public AutoRoutine testRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("Test Auto");
        AutoTrajectory driveToMiddle = routine.trajectory("TestPath");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Started the routine!"),

                        driveToMiddle.resetOdometry(),
                        driveToMiddle.cmd()));
        return routine;
    }

    public Command setAutoRoutine() {
        // return Commands.defer(routine.cmd()::init,routine.cmd()::execute,
        // routine.cmd()::end,(()->routine.cmd()::isFinished)), Set.of(drivetrain));
        AutoRoutine routine = autoFactory.newRoutine("Test Auto");
        AutoTrajectory driveToMiddle = routine.trajectory("TestPath");

        routine.active().onTrue(
                Commands.sequence(

                        driveToMiddle.resetOdometry(),
                        driveToMiddle.cmd()));
        return routine.cmd();
    }
}
