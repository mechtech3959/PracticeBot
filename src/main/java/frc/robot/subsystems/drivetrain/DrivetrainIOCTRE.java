package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.util.FieldBasedConstants;

// Inspired by FRC 2910 
public class DrivetrainIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DrivetrainIO {

    public DrivetrainIOCTRE(SwerveDrivetrainConstants constants,
            @SuppressWarnings("unchecked") SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants) {

        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);

        this.resetRotation(FieldBasedConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);

    }

    @Override
    public void registerDrivetrainTelemetry(DrivetrainIOInputs inputs) {
        this.registerTelemetry(state -> {

            SwerveDriveState modifiedState = (SwerveDriveState) state;
            modifiedState.Speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    ((SwerveDriveState) state).Speeds, ((SwerveDriveState) state).Pose.getRotation());
            inputs.logState(modifiedState);

        });
    }

    @Override
    public void updateDrivetrainData(DrivetrainIOInputs inputs) {
        var state = this.getStateCopy();
        state.Speeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        inputs.logState(state);
    }

    @Override
    public SwerveModule<TalonFX, TalonFX, CANcoder> getSwerveModule(int index) {
        return this.getModule(index);
    }

    @Override
    public void setSwerveState(SwerveRequest req) {
        this.setControl(req);
    }

    @Override
    public void resetHeading(Rotation2d Heading) {
        this.resetRotation(Heading);
    }

    @Override
    public ChassisSpeeds getRobotRelSpeed() {
        return this.getState().Speeds;

    }

    @Override
    public void trajPath(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public void resetRobotPose(Pose2d Pose) {
        this.resetPose(Pose);
    }

    @Override
    public void setPoseEstValues(Pose2d pose, double timestamp) {
        this.addVisionMeasurement(pose, timestamp);
    }
}
