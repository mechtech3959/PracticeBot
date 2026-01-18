package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;

public interface DrivetrainIO {

    public class DrivetrainIOInputs {
        public Pose2d Pose = new Pose2d();
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        public SwerveModuleState[] ModuleStates;
        public SwerveModuleState[] ModuleTargets;
        public SwerveModulePosition[] ModulePositions;
        public Rotation2d RawHeading = new Rotation2d();
        public double Timestamp;
        public double OdometryPeriod;
        public int SuccessfulDaqs;
        public int FailedDaqs;

        void logState(SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.RawHeading = state.RawHeading;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Speeds = state.Speeds;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
            this.OdometryPeriod = state.OdometryPeriod;
            this.Timestamp = state.OdometryPeriod;

        }
    }
        class ModuleIOInputs {
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveTemperature = 0.0;

        public double steerSupplyCurrentAmps = 0.0;
        public double steerStatorCurrentAmps = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerTemperature = 0.0;
    }

    default void configure() {
    }

    default void updateInputs(DrivetrainIOInputs inputs) {
    }

    default void resetHeading(Rotation2d heading) {
    }

    default void resetPose(Pose2d pose) {
    }

    default void simulationInit() {
    }

    default void periodic() {
    }

}
