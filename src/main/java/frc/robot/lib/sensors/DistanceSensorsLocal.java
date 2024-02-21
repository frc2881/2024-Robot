package frc.robot.lib.sensors;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensorsLocal {
  private Notifier m_notifier;
  private TCA9548A m_mux = new TCA9548A();
  private VL53L0X m_sensorIntake; 
  private VL53L0X m_sensorLauncher;
  private double m_sensorIntakeDistance = -1;
  private double m_sensorLauncherDistance = -1;

  public DistanceSensorsLocal() {
    m_mux.setEnabledBuses(0);
    m_sensorIntake = new VL53L0X(Port.kMXP);
    m_sensorIntake.startRanging();

    m_mux.setEnabledBuses(1);
    m_sensorLauncher = new VL53L0X(Port.kMXP);
    m_sensorLauncher.startRanging();

    m_notifier = new Notifier(() -> {
      m_mux.setEnabledBuses(0);
      Timer.delay(0.001);
      m_sensorIntakeDistance = m_sensorIntake.getDistance();
      m_mux.setEnabledBuses(1);
      Timer.delay(0.001);
      m_sensorLauncherDistance = m_sensorLauncher.getDistance();
    });
    m_notifier.startPeriodic(0.02);
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Sensor/Distance/Intake", m_sensorIntakeDistance);
    SmartDashboard.putNumber("Robot/Sensor/Distance/Launcher", m_sensorLauncherDistance);
  }
}
