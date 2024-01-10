package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.logging.Logger;
import frc.robot.lib.telemetry.Telemetry;

public class Robot extends TimedRobot {

  public static enum Mode { DISABLED, AUTO, TELEOP, TEST; }
  public static enum State { DISABLED, ENABLED, ESTOPPED; }

  private static Robot m_robotInstance;
  private RobotContainer m_robotContainer;
  private Command m_autoCommand;
  private Alliance m_alliance;

  @Override
  public void robotInit() {
    m_robotInstance = this;
    Logger.start();
    Telemetry.start(); 
    m_robotContainer = new RobotContainer();  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    DriverStation.getAlliance().ifPresent(alliance -> m_alliance = alliance);
  }

  @Override
  public void disabledInit() {
    Logger.log(Robot.Mode.DISABLED);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Logger.log(Robot.Mode.AUTO);
    m_autoCommand = m_robotContainer.getSelectedAutoCommand();
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
    m_robotContainer.reset();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Logger.log(Robot.Mode.TELEOP);
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
    if (!isRunningMatch()) {
      m_robotContainer.reset();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    Logger.log(Robot.Mode.TEST);
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.reset();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public static void addCustomPeriodic(Runnable callback, double periodSeconds) {
    m_robotInstance.addPeriodic(callback, periodSeconds, 0.333);
  }

  public static boolean isRunningMatch() {
    return DriverStation.getMatchTime() != -1;
  }
  
  public static Alliance getAlliance() {
    return m_robotInstance.m_alliance;
  }
}
