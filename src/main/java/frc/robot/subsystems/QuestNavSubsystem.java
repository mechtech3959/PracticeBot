package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    public Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            Math.toRadians(2) // Trust down to 2 degrees rotational
    );

    // Transform from robot center to Quest position
    // Quest is mounted 90° to the left (counterclockwise from robot forward)
    private static final Transform3d ROBOT_TO_QUEST = new Transform3d(
            0.022, // Forward from robot center (meters)
            0.152, // Left from robot center (meters)
            0.041, // Up from robot center (meters)
            new Rotation3d(0, 0, Math.toRadians(90)) // Quest faces 90° left relative to robot
    );

    private QuestNav questNav;
    private DrivetrainSubsystem drive;

    public QuestNavSubsystem(DrivetrainSubsystem drive) {
        this.drive = drive;
        questNav = new QuestNav();
    }

    public void trackingStart() {
        // Start robot at origin - you'll call setRobotPose() with actual position later
        Pose2d robotStartPose = new Pose2d(0, 0, new Rotation2d(0));
        setRobotPose(robotStartPose);
    }

    /**
     * Reset QuestNav using a robot pose (e.g., from AprilTags or auto start
     * position)
     */
    public void setRobotPose(Pose2d robotPose) {
        // Convert robot 2D pose to 3D
        Pose3d robotPose3d = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

        // Transform: Quest pose = Robot pose transformed by ROBOT_TO_QUEST
        Pose3d questPose = robotPose3d.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose);
    }

    /** Deprecated - use setRobotPose() instead */
    @Deprecated
    public void resetPose(Pose2d pose) {
        setRobotPose(pose);
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                // Get Quest pose from sensor in field frame
                Pose3d questPose = questFrame.questPose3d();
                double timestamp = questFrame.dataTimestamp();

                // To get robot pose from Quest pose, we need to subtract the transform
                // Think: if Quest is offset to the left of robot, robot is offset to the RIGHT
                // of Quest
                // So we apply the INVERSE transform
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                // Log for debugging
                Logger.recordOutput("QuestNav/RobotPose", robotPose.toPose2d());
                Logger.recordOutput("QuestNav/QuestPose", questPose.toPose2d());
                Logger.recordOutput("QuestNav/IsTracking", true);

                // Add measurement to pose estimator
                drive.poseEst(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            } else {
                Logger.recordOutput("QuestNav/IsTracking", false);
            }
        }
    }
}