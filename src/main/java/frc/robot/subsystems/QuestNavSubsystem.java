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
import frc.robot.subsystems.util.FieldBasedConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    public Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );
    // 0.41
    private QuestNav questNav;
    public Transform3d ROBOT_TO_QUEST;// = new Transform3d(0.022, 0.152, 0.041, new Rotation3d(new Rotation2d(-90)));
    private DrivetrainSubsystem drive;
    public Pose3d robotPose; 

    public QuestNavSubsystem(DrivetrainSubsystem drive) {

        this.drive = drive;
        ROBOT_TO_QUEST = new Transform3d(0.022, 0.152, 0.041,
                new Rotation3d(new Rotation2d(drive.getHeading().getDegrees()).plus(new Rotation2d(-90))));
        questNav = new QuestNav();

        Pose3d questPose = new Pose3d(0, 0, 0, new Rotation3d(new Rotation2d(-90)));
        // Transform by the mount pose to get your robot pose
          robotPose = questPose.transformBy(ROBOT_TO_QUEST);

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
                  robotPose = questPose.transformBy(ROBOT_TO_QUEST);

                Logger.recordOutput("QuestNav23", robotPose.toPose2d());
                // You can put some sort of filtering here if you would like!
                drive.poseEst(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
                // Add the measurement to our estimator
            }
        }
    }
}
