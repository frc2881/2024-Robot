package frc.robot.lib.telemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public final class Telemetry {
  private static boolean m_isAllTelemetryEnabled = false;

  public static void start() {
    Robot.addCustomPeriodic(Telemetry::updateTimingInfo, 0.3);
    Robot.addCustomPeriodic(Telemetry::updateRobotInfo, 1);
    Robot.addCustomPeriodic(Telemetry::updateTelemetrySetting, 3.0);
  }

  private static void updateTimingInfo() {
    SmartDashboard.putNumber("Timing/FPGATimestamp", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Timing/MatchTime", Math.floor(DriverStation.getMatchTime()));
  }

  private static void updateRobotInfo() {
    Robot.Mode mode = Robot.Mode.DISABLED;
    if (RobotState.isAutonomous()) { mode = Robot.Mode.AUTO; }
    if (RobotState.isTeleop()) { mode = Robot.Mode.TELEOP; }
    if (RobotState.isTest()) { mode = Robot.Mode.TEST; }
    SmartDashboard.putString("Robot/Mode", mode.toString());

    Robot.State state = Robot.State.DISABLED;
    if (RobotState.isEnabled()) { state = Robot.State.ENABLED; }
    if (RobotState.isEStopped()) { state = Robot.State.ESTOPPED; }
    SmartDashboard.putString("Robot/State", state.toString());

    SmartDashboard.putNumber("Robot/Battery/Voltage", RobotController.getBatteryVoltage());
  }

  private static void updateTelemetrySetting() {
    if (Robot.isRunningMatch()) {
      if (m_isAllTelemetryEnabled) {
        m_isAllTelemetryEnabled = false;
        LiveWindow.disableAllTelemetry();
        SmartDashboard.putBoolean("IsAllTelemetryEnabled", false);
      }
    } else {
      boolean isAllTelemetryEnabled = SmartDashboard.getBoolean("IsAllTelemetryEnabled", m_isAllTelemetryEnabled);
      if (m_isAllTelemetryEnabled != isAllTelemetryEnabled) {
        m_isAllTelemetryEnabled = isAllTelemetryEnabled;
        if (m_isAllTelemetryEnabled) {
          LiveWindow.enableAllTelemetry();
        } else {
          LiveWindow.disableAllTelemetry();
        }
      } 
      SmartDashboard.putBoolean("IsAllTelemetryEnabled", m_isAllTelemetryEnabled);     
    }   
  }
}
