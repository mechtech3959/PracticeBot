package frc.robot.subsystems.drivetrain;

import java.io.Console;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.util.BaseCalculator;
import frc.robot.subsystems.util.FieldBasedConstants;

import frc.robot.subsystems.drivetrain.DrivetrainIO;

public class DrivetrainSubsystem extends SubsystemBase {

    public enum SwerveState {
        Disabled,
        Brake,
        Pathing,
        TeliOp,
        Slow,
        Heading,
        VisionHeading,

    }
    public enum OutputMode{
        Fast,
        Slow,
        Ramp
    }
    private final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(3, 0, 0)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private SwerveState currentDriveState = SwerveState.TeliOp;
    private CommandXboxController controller;
    private DrivetrainIO io = new DrivetrainIO() {
    };
    final DrivetrainIOInputsAutoLogged swerveInputs = new DrivetrainIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

    double maxSpeed;
    double maxAngSpeed;

    public DrivetrainSubsystem(DrivetrainIO io, CommandXboxController controller) {
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
        // xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        // yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (FieldBasedConstants.isBlueAlliance() ? -xMagnitude * 4.5 : xMagnitude * 4.5)
                * 1.0;
        double yVelocity = (FieldBasedConstants.isBlueAlliance() ? -yMagnitude * 4.5 : yMagnitude * 4.5)
                * 1.0;
        double angularVelocity = angularMagnitude * 1.75 * 1.75;

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(swerveInputs.Speeds.omegaRadiansPerSecond * -0.03);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), swerveInputs.Pose.getRotation()),
                swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
    }

    private ChassisSpeeds slowcalculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), 0.1);
        double yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
        double angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), 0.1);
        Logger.recordOutput("xMag", xMagnitude);
        //
        // xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        // yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (FieldBasedConstants.isBlueAlliance() ? -xMagnitude * 2 : xMagnitude * 2)
                * 0.8;
        double yVelocity = (FieldBasedConstants.isBlueAlliance() ? -yMagnitude * 2 : yMagnitude * 2)
                * 0.8;
        double angularVelocity = angularMagnitude * 1 * 1;

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(swerveInputs.Speeds.omegaRadiansPerSecond * -0.03);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), swerveInputs.Pose.getRotation()),
                swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
    }

    public void teliopDrive() {
        io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(calculateSpeedsBasedOnJoystickInputs())
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }

    public void brake() {
        io.setSwerveState(new SwerveRequest.SwerveDriveBrake());
    }

    public void disable() {
    }

    public void headingDrive() {
        io.setSwerveState(headingDrive.withTargetDirection(BaseCalculator.angleToAlign(swerveInputs.Pose))
                .withVelocityX(calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
                .withVelocityY(calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond));

    }

    public void slowDrive() {
        io.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(slowcalculateSpeedsBasedOnJoystickInputs())
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }

    public void autoPath(ChassisSpeeds speeds) {
        io.setSwerveState(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public void visionHeadingDrive() {
    }

    public void applyState() {
        switch (currentDriveState) {
            case Disabled:
                disable();
                break;
            case Brake:
                brake();
                break;
            case Pathing:

                break;
            case TeliOp:
                teliopDrive();
                break;
            case Slow:
                slowDrive();
                break;
            case Heading:
                headingDrive();
                break;
            case VisionHeading:
                visionHeadingDrive();
                break;

            default:

                break;
        }

    }

    public void changeState(SwerveState wanted) {
        currentDriveState = wanted;
    }

    @Override
    public void periodic() {

        io.updateDrivetrainData(swerveInputs);
        Logger.processInputs(getName(), swerveInputs);
        // teliopDrive();
        // headingDrive();
        applyState();
    }

}
