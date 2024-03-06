package frc.robot.lib.sensors;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;

public class PoseSensor {
  private final PhotonCamera m_photonCamera;
  private final PhotonPoseEstimator m_photonPoseEstimator;
  private final Matrix<N3, N1> m_singleTagStandardDeviations;
  private final Matrix<N3, N1> m_multiTagStandardDeviations; 

  public PoseSensor(
    String cameraName, 
    Transform3d cameraTransform, 
    PoseStrategy poseStrategy, 
    PoseStrategy fallbackPoseStrategy,
    Matrix<N3, N1> singleTagStandardDeviations,
    Matrix<N3, N1> multiTagStandardDeviations, 
    AprilTagFieldLayout aprilTagFieldLayout
  ) {
    m_photonCamera = new PhotonCamera(cameraName);
    m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, m_photonCamera, cameraTransform);
    m_photonPoseEstimator.setMultiTagFallbackStrategy(fallbackPoseStrategy);
    m_singleTagStandardDeviations = singleTagStandardDeviations;
    m_multiTagStandardDeviations = multiTagStandardDeviations; 
  }

  private PhotonPipelineResult getLatestResult() {
    return m_photonCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return m_photonPoseEstimator.update();
  }

  public Matrix<N3, N1> getEstimatedStandardDeviations(Pose2d estimatedPose) {
    Matrix<N3, N1> estimatedStandardDeviations = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
    int tagCount = 0;
    double averageDistance = 0;
    for (var target : targets) {
      if (Utils.isValueBetween(target.getPoseAmbiguity(), 0, Constants.Sensors.Pose.kMaxTargetPoseAmbiguity)) {
        var tagPose = m_photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
        if (tagPose.isPresent()) {
          tagCount++;
          averageDistance += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
      }
    }
    if (tagCount > 0) {
      estimatedStandardDeviations = tagCount > 1 ? m_multiTagStandardDeviations : m_singleTagStandardDeviations;
      averageDistance /= tagCount;
      if (averageDistance <= Constants.Sensors.Pose.kMaxTargetsAverageDistance) {
        estimatedStandardDeviations = estimatedStandardDeviations.times(1 + (averageDistance * averageDistance / 30));
      }
    }
    return estimatedStandardDeviations;
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/Pose/" + m_photonCamera.getName() + "/HasTargets", m_photonCamera.getLatestResult().hasTargets());
  }
}
