package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DrivetrainIO {

    @AutoLog
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

    @AutoLog
    class ModuleIOInputs {
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveTemperature = 0.0;

        public double steerSupplyCurrentAmps = 0.0;
        public double steerStatorCurrentAmps = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerTemperature = 0.0;
        public void updateModuleData(){}
    }

    default void registerDrivetrainTelemetry(DrivetrainIOInputs inputs) {
    }

    default void updateDrivetrainData(DrivetrainIOInputs inputs) {
    }

    default void updateModuleData(ModuleIOInputs inputs) {
    }

    default void configure() {
    }

    default void updateInputs(DrivetrainIOInputs inputs) {
    }

    default void resetHeading(Rotation2d heading) {
    }
    default Pose2d getPose(){
        return new Pose2d();
    }

    default void resetPose(Pose2d pose) {
    }

    default void setSwerveState(SwerveRequest req) {
    }

    default void setPoseEstValues(Pose2d pose, double timestamp) {
    }
    default ChassisSpeeds getRobotRelSpeed(){return new ChassisSpeeds();}
    default void autoPath(ChassisSpeeds speeds){};

    default void simulationInit() {
    }

    default void periodic() {
    }

}
