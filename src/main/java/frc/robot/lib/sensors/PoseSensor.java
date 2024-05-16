package frc.robot.lib.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseSensor {
  private final PhotonCamera m_photonCamera;
  private final PhotonPoseEstimator m_photonPoseEstimator;

  public PoseSensor(
    String cameraName, 
    Transform3d cameraTransform, 
    PoseStrategy poseStrategy, 
    PoseStrategy fallbackPoseStrategy,
    AprilTagFieldLayout aprilTagFieldLayout
  ) {
    m_photonCamera = new PhotonCamera(cameraName);
    m_photonCamera.setDriverMode(false);
    m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, m_photonCamera, cameraTransform);
    m_photonPoseEstimator.setMultiTagFallbackStrategy(fallbackPoseStrategy);
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    return m_photonPoseEstimator.update();
  }

  public boolean hasTarget() {
    return m_photonCamera.getLatestResult().hasTargets();
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/Pose/" + m_photonCamera.getName() + "/IsConnected", m_photonCamera.isConnected());
    SmartDashboard.putBoolean("Robot/Sensor/Pose/" + m_photonCamera.getName() + "/HasTarget", m_photonCamera.getLatestResult().hasTargets());
  }
}
