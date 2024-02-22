package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreakSensor {
  private final String m_sensorName;
  private final DigitalInput m_digitalInput;

  public BeamBreakSensor(String sensorName, int channel) {
    m_sensorName = sensorName;
    m_digitalInput = new DigitalInput(channel);
  }

  public boolean hasTarget() {
    return !m_digitalInput.get();
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/BeamBreak/" + m_sensorName + "/HasTarget", hasTarget());
  }
}
