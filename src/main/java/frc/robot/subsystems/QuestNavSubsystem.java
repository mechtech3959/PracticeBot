package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.util.FieldBasedConstants;
import gg.questnav.questnav.*;

public class QuestNavSubsystem extends SubsystemBase {
    private QuestNav questNav;

    public QuestNavSubsystem() {

        questNav = new QuestNav();
        Transform3d ROBOT_TO_QUEST = new Transform3d(0.0028, 0.008, 0.05, new Rotation3d(new Rotation2d(0)));
        Pose3d questPose = new Pose3d(0, 0, 0, new Rotation3d(new Rotation2d(0)));
        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(robotPose);

    }

    public void trackingStart() {

        questNav.setPose(new Pose3d(0, 0, 0, new Rotation3d(
                new Rotation2d((FieldBasedConstants.isBlueAlliance()) ? 0 : 180))));

    }

    public void resetPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose));
    }

    @Override
    public void periodic() {

        questNav.commandPeriodic();
        Transform3d ROBOT_TO_QUEST = new Transform3d(0.0028, 0.008, 0.05, new Rotation3d(new Rotation2d(-90)));
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
                Logger.recordOutput("QuestNav2d", robotPose.toPose2d());
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                // Pose3d robotPose =
                // questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
            }

        }
    }
}
