package frc.robot.lib.sensors;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectSensor {
  private final PhotonCamera m_photonCamera;

  public ObjectSensor(
    String cameraName
  ) {
    m_photonCamera = new PhotonCamera(cameraName);
  }

  private PhotonPipelineResult getLatestResult() {
    return m_photonCamera.getLatestResult();
  }

  public Optional<PhotonTrackedTarget> getTrackedObject() {
    PhotonPipelineResult result = getLatestResult();
    if (result.hasTargets()) {
      return Optional.of(result.getBestTarget());
    }
    return Optional.empty();
  }
}
