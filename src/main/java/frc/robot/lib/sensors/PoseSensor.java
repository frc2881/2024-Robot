package frc.robot.lib.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
    m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, m_photonCamera, cameraTransform);
    m_photonPoseEstimator.setMultiTagFallbackStrategy(fallbackPoseStrategy);
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    return m_photonPoseEstimator.update();
  }

  public double getLatestResultPoseAmbiguity() {
    double poseAmbiguity = -1;
    PhotonPipelineResult result = m_photonCamera.getLatestResult();
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      poseAmbiguity = m_photonCamera.getLatestResult().getMultiTagResult().estimatedPose.ambiguity;
    } else {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target != null) {
        poseAmbiguity = target.getPoseAmbiguity();
      }
    }
    return poseAmbiguity;
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/Pose/" + m_photonCamera.getName() + "/HasTargets", m_photonCamera.getLatestResult().hasTargets());
    SmartDashboard.putNumber("Robot/Sensor/Pose/" + m_photonCamera.getName() + "/Ambiguity", getLatestResultPoseAmbiguity());
  }
}
