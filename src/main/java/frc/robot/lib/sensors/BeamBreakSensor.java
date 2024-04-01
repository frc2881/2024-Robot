package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreakSensor {
  private final String m_sensorName;
  private final DigitalInput m_digitalInput;
  private boolean m_isTriggered = false;

  public BeamBreakSensor(String sensorName, int channel) {
    m_sensorName = sensorName;
    m_digitalInput = new DigitalInput(channel);
  }

  public boolean hasTarget() {
    boolean hasTarget = !m_digitalInput.get();
    if (hasTarget && !m_isTriggered) {
      m_isTriggered = true;
    }
    return hasTarget;
  }

  public boolean isTriggered() {
    return m_isTriggered;
  }

  public void resetTrigger() {
    m_isTriggered = false;
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/BeamBreak/" + m_sensorName + "/HasTarget", hasTarget());
    SmartDashboard.putBoolean("Robot/Sensor/BeamBreak/" + m_sensorName + "/IsTriggered", isTriggered());
  }
}
