package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.util.BaseCalculator;
import frc.robot.subsystems.util.FieldBasedConstants;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class DrivetrainSubsystem extends SubsystemBase {

    public enum SwerveState {
        Disabled,
        Brake,
        ChoreoTrajectory,
        TeliOp,
        Slow,
        Heading,
        VisionHeading,

    }

    public enum OutputMode {
        Fast,
        Slow,
        Ramp
    }

    private final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(3, 0, 0)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final PIDController autoXController = new PIDController(7, 0, 0);
    private final PIDController autoYController = new PIDController(7, 0, 0);
    private final PIDController autoHeadingController = new PIDController(7, 0, 0);
    private SwerveSample trajectorySample;
    private final PIDController autoDriveController = new PIDController(3.0, 0, 0.1);

    private SwerveState currentDriveState = SwerveState.TeliOp;
    private OutputMode selectedSpeed = OutputMode.Fast;
    private CommandXboxController controller;
    private DrivetrainIO io = new DrivetrainIO() {
    };
    final DrivetrainIOInputsAutoLogged swerveInputs = new DrivetrainIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {
            new ModuleIOInputsAutoLogged(), // FL (Index 0)
            new ModuleIOInputsAutoLogged(), // FR (Index 1)
            new ModuleIOInputsAutoLogged(), // BL (Index 2)
            new ModuleIOInputsAutoLogged() // BR (Index 3)
    };

    private final ModuleIO[] modules = new ModuleIO[4];

    double maxSpeed;
    double maxAngSpeed;

    public DrivetrainSubsystem(DrivetrainIO io, CommandXboxController controller) {
        this.io = io;
        this.controller = controller;

        for (int i = 0; i < 4; i++) {
            modules[i] = new ModuleIOCTRE(io.getSwerveModule(i));
            modules[i].updateInputs(moduleInputs[i]);

        }

        io.registerDrivetrainTelemetry(swerveInputs);

        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public void poseEst(Pose2d pose,double  time,Matrix<N3,N1> dev ){

        io.setPoseEstValues(pose, time,dev);
    }
    // TODO: Decide what im going to do with this / find out if it works and if it
    // is worth it to combine code or seperate
    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        // was .isEmpty() but threw error for some reason
        if (!DriverStation.getAlliance().isPresent()) {
            return new ChassisSpeeds(0, 0, 0);
        }
        switch (selectedSpeed) {
            case Fast:
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

                Rotation2d skewCompensationFactor = Rotation2d
                        .fromRadians(swerveInputs.Speeds.omegaRadiansPerSecond * -0.03);

                return ChassisSpeeds.fromRobotRelativeSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity),
                                swerveInputs.Pose.getRotation()),
                        swerveInputs.Pose.getRotation().plus(skewCompensationFactor));

            case Slow:
                xMagnitude = MathUtil.applyDeadband(controller.getLeftY(), 0.1);
                yMagnitude = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
                angularMagnitude = MathUtil.applyDeadband(controller.getRightX(), 0.1);
                Logger.recordOutput("xMag", xMagnitude);
                //
                // xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
                // yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
                angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

                xVelocity = (FieldBasedConstants.isBlueAlliance() ? -xMagnitude * 2 : xMagnitude * 2)
                        * 0.8;
                yVelocity = (FieldBasedConstants.isBlueAlliance() ? -yMagnitude * 2 : yMagnitude * 2)
                        * 0.8;
                angularVelocity = angularMagnitude * 1 * 1;

                skewCompensationFactor = Rotation2d.fromRadians(swerveInputs.Speeds.omegaRadiansPerSecond * -0.03);

                return ChassisSpeeds.fromRobotRelativeSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity),
                                swerveInputs.Pose.getRotation()),
                        swerveInputs.Pose.getRotation().plus(skewCompensationFactor));
            case Ramp:
                return new ChassisSpeeds(0, 0, 0);

            default:
                return new ChassisSpeeds(0, 0, 0);

        }
    }

    private ChassisSpeeds slowcalculateSpeedsBasedOnJoystickInputs() {
        // was .isEmpty() but threw error for some reason
        if (!DriverStation.getAlliance().isPresent()) {
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

    public Pose2d getPose() {
        return io.getPose();
    }
    public void resetPose(Pose2d pose) {
        io.resetRobotPose(pose);
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = io.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + autoXController.calculate(pose.getX(), sample.x),
                sample.vy + autoYController.calculate(pose.getY(), sample.y),
                sample.omega + autoHeadingController.calculate(pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        io.trajPath(speeds);
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
            case ChoreoTrajectory:
                followTrajectory(trajectorySample);
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
    public void stageTrajectory(SwerveSample sample) {
        trajectorySample = sample;
        currentDriveState = SwerveState.ChoreoTrajectory;
    }

    public Rotation2d getHeading() {
        return swerveInputs.Pose.getRotation();
    }   
    @Override
    public void periodic() {

        io.updateDrivetrainData(swerveInputs);
        Logger.processInputs(getName() + "/Swerve", swerveInputs);
        // teliopDrive();
        // headingDrive();
        applyState();

              modules[0].updateInputs(moduleInputs[0]);
              modules[1].updateInputs(moduleInputs[1]);
              modules[2].updateInputs(moduleInputs[2]);
              modules[3].updateInputs(moduleInputs[3]);

            // Send to dashboard
            Logger.processInputs("Drive/Module 0", moduleInputs[0]);      
            // Send to dashboard
            Logger.processInputs("Drive/Module 1", moduleInputs[1]);      
            Logger.processInputs("Drive/Module  2", moduleInputs[2]);     
            // Send to dashboard
            Logger.processInputs("Drive/Module 3", moduleInputs[3]);
       /*  for (int i = 0; i < 4; i++) {
            // Read fresh data from hardware
            modules[i].updateInputs(moduleInputs[i]);
            // Send to dashboard
            Logger.processInputs("Drive/Module " + i, moduleInputs[i]);
        }*/
    }
}
