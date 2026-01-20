package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

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
    
        private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), 0.1);
        double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
        double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), 0.1);
        Logger.recordOutput("xMag", xMagnitude);
        //
        //        xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        //        yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (FieldBasedConstants.isBlueAlliance() ? -xMagnitude * 4.5 : xMagnitude * 4.5)
                * 1.0;
        double yVelocity = (FieldBasedConstants.isBlueAlliance() ? -yMagnitude * 4.5 : yMagnitude * 4.5)
                * 1.0;
        double angularVelocity = angularMagnitude * 1.75 * 1.75;

        Rotation2d skewCompensationFactor =
                Rotation2d.fromRadians( swerveInputs.Speeds.omegaRadiansPerSecond * -0.03);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), swerveInputs.Pose.getRotation()),
                swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
    }
    public void teliopDrive(){
        io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                        .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }
    @Override
    public void periodic() {

        io.updateDrivetrainData(swerveInputs);
        Logger.processInputs(getName(), swerveInputs);
        teliopDrive();
    }

}
