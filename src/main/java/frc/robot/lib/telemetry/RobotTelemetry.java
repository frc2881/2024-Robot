package frc.robot.lib.telemetry;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public final class RobotTelemetry {
  private static boolean m_isAllTelemetryEnabled = false;

  public static void start() {
    Robot.addCustomPeriodic(RobotTelemetry::updateTimingInfo, 0.3);
    Robot.addCustomPeriodic(RobotTelemetry::updateRobotInfo, 1);
    Robot.addCustomPeriodic(RobotTelemetry::updateTelemetrySetting, 3.0);
    getGameFieldLayoutData();
  }

  private static void updateTimingInfo() {
    SmartDashboard.putNumber("Robot/FPGATimestamp", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("Robot/Game/MatchTime", Math.floor(DriverStation.getMatchTime()));
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

    SmartDashboard.putNumber("Robot/Power/Battery/Voltage", RobotController.getBatteryVoltage());

    SmartDashboard.putString("Robot/Game/Alliance", Robot.getAlliance().toString().toUpperCase());
    SmartDashboard.putNumber("Robot/Game/StationNumber", DriverStation.getLocation().orElse(0));
  }

  private static void updateTelemetrySetting() {
    if (Robot.isRunningMatch()) {
      if (m_isAllTelemetryEnabled) {
        m_isAllTelemetryEnabled = false;
        LiveWindow.disableAllTelemetry();
        SmartDashboard.putBoolean("Robot/IsAllTelemetryEnabled", false);
      }
    } else {
      boolean isAllTelemetryEnabled = SmartDashboard.getBoolean("Robot/IsAllTelemetryEnabled", m_isAllTelemetryEnabled);
      if (m_isAllTelemetryEnabled != isAllTelemetryEnabled) {
        m_isAllTelemetryEnabled = isAllTelemetryEnabled;
        if (m_isAllTelemetryEnabled) {
          LiveWindow.enableAllTelemetry();
        } else {
          LiveWindow.disableAllTelemetry();
        }
      } 
      SmartDashboard.putBoolean("Robot/IsAllTelemetryEnabled", m_isAllTelemetryEnabled);     
    }   
  }

  private static void getGameFieldLayoutData() {
    try {
      Path filePath = Paths.get(Filesystem.getOperatingDirectory().getPath() + "/april-tag-field-layout.json");
      Constants.Game.Field.kAprilTagFieldLayout.serialize(filePath);
      SmartDashboard.putString("Robot/Game/Field", new String(Files.readAllBytes(filePath), StandardCharsets.UTF_8));
      Files.delete(filePath);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
