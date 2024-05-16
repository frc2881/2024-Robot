package frc.robot.lib.sensors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ObjectSensor {
  private final PhotonCamera m_photonCamera;
  private final String m_topicName;

  public ObjectSensor(String cameraName, String objectName) {
    m_photonCamera = new PhotonCamera(cameraName);
    m_topicName = "Robot/Sensor/Object/" + m_photonCamera.getName() + "/" + objectName;
  }

  private PhotonPipelineResult getLatestResult() {
    return m_photonCamera.getLatestResult();
  }

  public boolean hasTarget() {
    return m_photonCamera.getLatestResult().hasTargets();
  }

  public double getTargetYaw() {
    PhotonPipelineResult result = getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getYaw() : Double.NaN;
  }

  public double getTargetArea() {
    PhotonPipelineResult result = getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getArea() : Double.NaN;
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean(m_topicName + "/IsConnected", m_photonCamera.isConnected());
    SmartDashboard.putBoolean(m_topicName + "/HasTarget", hasTarget());
    SmartDashboard.putNumber(m_topicName + "/Target/Yaw", getTargetYaw());
    SmartDashboard.putNumber(m_topicName + "/Target/Area", getTargetArea());
  }
}
