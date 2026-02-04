package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;

import choreo.auto.AutoTrajectory;
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
                drivetrain::stageTrajectory,
                true,
                drivetrain

        );

    }

    public AutoRoutine testRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("Test Auto");
        AutoTrajectory driveToMiddle = routine.trajectory("TestPath");

        routine.active().onTrue(
                Commands.sequence(
                        driveToMiddle.resetOdometry(),
                        driveToMiddle.cmd()));
        return routine;
    }
}
