package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor {
  private String m_topicName;
  
  public DistanceSensor(
    String sensorName
  ) {
    m_topicName = "Robot/Sensor/Distance/" + sensorName;
  }

  public Double getDistance() {
    return SmartDashboard.getEntry(m_topicName).getDouble(-1.0);
  }
}
