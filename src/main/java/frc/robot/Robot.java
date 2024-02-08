package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.common.Enums.RobotMode;
import frc.robot.lib.common.Enums.RobotState;
import frc.robot.lib.logging.Logger;
import frc.robot.lib.telemetry.RobotTelemetry;

public class Robot extends TimedRobot {
  private static Robot m_robotInstance;
  private RobotContainer m_robotContainer;
  private Command m_autoCommand;

  @Override
  public void robotInit() {
    m_robotInstance = this;
    Logger.start();
    RobotTelemetry.start(); 
    m_robotContainer = new RobotContainer();  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateTelemetry();
  }

  @Override
  public void disabledInit() {
    Logger.log(RobotMode.Disabled);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Logger.log(RobotMode.Auto);
    m_autoCommand = m_robotContainer.getSelectedAutoCommand();
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Logger.log(RobotMode.Teleop);
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    Logger.log(RobotMode.Test);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    Logger.log(RobotMode.Sim);
  }

  @Override
  public void simulationPeriodic() {}

  public static void addCustomPeriodic(Runnable callback, double periodSeconds) {
    m_robotInstance.addPeriodic(callback, periodSeconds, 0.333);
  }

  public static RobotMode getMode() {
    if (edu.wpi.first.wpilibj.RobotState.isTeleop()) {
      return RobotMode.Teleop;
    } else if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      return RobotMode.Auto;
    } else if (edu.wpi.first.wpilibj.RobotState.isTest()) {
      return RobotMode.Test;
    } else {
      return RobotMode.Disabled;
    }
  }

  public static RobotState getState() {
    if (edu.wpi.first.wpilibj.RobotState.isEnabled()) {
      return RobotState.Enabled;
    } else if (edu.wpi.first.wpilibj.RobotState.isEStopped()) {
      return RobotState.EStopped;
    } else {
      return RobotState.Disabled;
    }
  }

  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public static boolean isRunningMatch() {
    return DriverStation.getMatchTime() != -1;
  }
}
