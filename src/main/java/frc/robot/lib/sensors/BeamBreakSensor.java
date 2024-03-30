package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreakSensor {
  private final String m_sensorName;
  private final DigitalInput m_digitalInput;
  private boolean m_hasInitialTarget = false;

  public BeamBreakSensor(String sensorName, int channel) {
    m_sensorName = sensorName;
    m_digitalInput = new DigitalInput(channel);
  }

  public boolean hasTarget() {
    boolean hasTarget = !m_digitalInput.get();
    if(hasTarget && !m_hasInitialTarget){
      m_hasInitialTarget = true;
    }
    return hasTarget;
  }

  public boolean hasInitialTarget() {
    return m_hasInitialTarget;
  }

  public void clearInitialTarget() {
    m_hasInitialTarget = false;
  }

  public void updateTelemetry() {
    SmartDashboard.putBoolean("Robot/Sensor/BeamBreak/" + m_sensorName + "/HasTarget", hasTarget());
  }
}
