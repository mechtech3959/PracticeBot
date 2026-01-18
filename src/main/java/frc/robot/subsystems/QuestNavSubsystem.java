package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import  gg.questnav.questnav.QuestNav;
import  gg.questnav.questnav.*;


public class QuestNavSubsystem extends SubsystemBase{
    private QuestNav questNav;

public QuestNavSubsystem(){

    questNav = new QuestNav();
}
public void trackingStart(){

    questNav.setPose(new Pose3d(0,0,0, new Rotation3d(new Rotation2d(180))));
    
}

@Override
public void periodic(){

    questNav.commandPeriodic();

      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
        // Make sure the Quest was tracking the pose for this frame
        if (questFrame.isTracking()) {
            // Get the pose of the Quest
            Pose3d questPose = questFrame.questPose3d();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
           // Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
}

}
}
}
