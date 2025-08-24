package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();
  private Pose2d lastRobotPose = new Pose2d();

  // Replace with your headset mount’s offset from robot center
  private static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.2, 0.0, new Rotation2d(0));

  // Standard deviations for vision trust
  private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // X trust (meters)
          0.02, // Y trust (meters)
          0.035 // Rotation trust (radians, ~2 degrees)
          );

  private final Drive swerveDriveSubsystem;

  public QuestNavSubsystem(Drive swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  @Override
  public void periodic() {
    // Required for QuestNav to update
    questNav.commandPeriodic();

    if (questNav.isTracking()) {
      // Get all new pose frames
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

      for (PoseFrame frame : poseFrames) {
        Pose2d questPose = frame.questPose();
        double timestamp = frame.dataTimestamp();

        // Transform into robot space
        Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

        // Store latest pose for logging/telemetry
        lastRobotPose = robotPose;

        // Feed into estimator
        swerveDriveSubsystem.addVisionMeasurement(robotPose, timestamp, QUESTNAV_STD_DEVS);
      }
    }

    // Log latest pose
    Logger.recordOutput("QuestNav/RobotPose", lastRobotPose);
    Logger.recordOutput("QuestNav/Conenected", questNav.isConnected());
    System.out.println(lastRobotPose);
  }

  public Pose2d getRobotPose() {
    return lastRobotPose;
  }
}
