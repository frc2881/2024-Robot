package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.common.Utils;

public class DistanceSensor {
  private String m_sensorName;
  private double m_minTargetDistance;
  private double m_maxTargetDistance;

  private String m_topicName;
  
  public DistanceSensor(
    String sensorName,
    double minTargetDistance,
    double maxTargetDistance
  ) {
    m_sensorName = sensorName;
    m_minTargetDistance = minTargetDistance;
    m_maxTargetDistance = maxTargetDistance;

    m_topicName = "Robot/Sensor/Distance/" + m_sensorName;
  }

  public double getDistance() {
    return SmartDashboard.getEntry(m_topicName).getDouble(-1.0);
  }

  public boolean hasTarget() {
    return Utils.isValueBetween(getDistance(), m_minTargetDistance, m_maxTargetDistance);
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean(m_topicName + "/HasTarget", hasTarget());
  }
}
