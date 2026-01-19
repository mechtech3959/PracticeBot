package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.util.FieldBasedConstants;

import frc.robot.subsystems.drivetrain.DrivetrainIO;

public class DrivetrainSubsystem extends SubsystemBase {

    private CommandXboxController controller;
    private DrivetrainIO io = new DrivetrainIO() {};
     final DrivetrainIOInputsAutoLogged swerveInputs = new DrivetrainIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

    double maxSpeed;
    double maxAngSpeed;
    public DrivetrainSubsystem(DrivetrainIO io, CommandXboxController controller){
        this.io = io;
        this.controller = controller;
    
    }
    /* *
        private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADBAND);
        double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADBAND);
        double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADBAND);

        //
        //        xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        //        yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (FieldBasedConstants.isBlueAlliance() ? -xMagnitude * 4.5 : xMagnitude * 4.5)
                * 4.5;
        double yVelocity = (FieldBasedConstants.isBlueAlliance() ? -yMagnitude * 4.5 : yMagnitude * 4.5)
                * 4.5;
        double angularVelocity = angularMagnitude * 0.75 * 0.75;

        Rotation2d skewCompensationFactor =
                Rotation2d.fromRadians( * -0.03);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), swerveInputs.Pose.getRotation()),
                swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
    }*/
    @Override
    public void periodic() {
    }

}
