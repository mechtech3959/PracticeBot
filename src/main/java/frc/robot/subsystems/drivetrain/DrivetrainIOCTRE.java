package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.util.FieldBasedConstants;

// Inspired by FRC 2910 
public class DrivetrainIOCTRE extends SwerveDrivetrain implements DrivetrainIO {

    private SwerveRequest.ApplyRobotSpeeds pathApplyingRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    @SuppressWarnings("unchecked")
    public DrivetrainIOCTRE(SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants) {

        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);

        this.resetRotation(FieldBasedConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);

    }

    public void configureAutoBuilder() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> getState().Pose, // Robot pose supplier
                 this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(
                        pathApplyingRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), // Method
                                                                                                           // that will
                                                                                                           // drive the
                                                                                                           // robot
                                                                                                           // given
                                                                                                           // ROBOT
                // RELATIVE ChassisSpeeds. Also optionally outputs
                // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    @SuppressWarnings("unchecked")
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
        inputs.logState(getState());

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
    public void autoPath(ChassisSpeeds speeds) {
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public void resetPose(Pose2d Pose) {
        this.resetPose(Pose);
    }

    @Override
    public void setPoseEstValues(Pose2d pose, double timestamp) {
        this.addVisionMeasurement(pose, timestamp);
    }
}
