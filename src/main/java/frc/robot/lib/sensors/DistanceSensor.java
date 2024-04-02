package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.common.Utils;

public class DistanceSensor {
  private final String m_sensorName;
  private final double m_minTargetDistance;
  private final double m_maxTargetDistance;
  private final String m_topicName;
  
  public DistanceSensor(String sensorName, double minTargetDistance, double maxTargetDistance) {
    m_sensorName = sensorName;
    m_minTargetDistance = minTargetDistance;
    m_maxTargetDistance = maxTargetDistance;
    m_topicName = "Robot/Sensor/Distance/" + m_sensorName;
  }

  public double getDistance() {
    return SmartDashboard.getEntry(m_topicName).getDouble(-1.0);
  }

  public boolean hasTarget() {
    return Utils.isValueInRange(getDistance(), m_minTargetDistance, m_maxTargetDistance);
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean(m_topicName + "/HasTarget", hasTarget());
  }
}
