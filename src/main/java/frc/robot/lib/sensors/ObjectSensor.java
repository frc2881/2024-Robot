package frc.robot.lib.sensors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public boolean hasTargets() {
    return m_photonCamera.getLatestResult().hasTargets();
  }

  public double getTargetYaw() {
    PhotonPipelineResult result = getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getYaw() : Double.NaN;
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/Object/" + m_photonCamera.getName() + "/HasTargets", hasTargets());
    SmartDashboard.putNumber("Robot/Sensor/Object/" + m_photonCamera.getName() + "/Target/Yaw", getTargetYaw());
  }
}
