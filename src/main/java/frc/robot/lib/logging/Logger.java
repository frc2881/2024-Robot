package frc.robot.lib.logging;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Robot;
import frc.robot.lib.common.Enums.RobotMode;

public final class Logger {

  static {
    DataLogManager.start();
    
    DriverStation.startDataLog(DataLogManager.getLog());

    CommandScheduler.getInstance().onCommandInitialize(command -> log("----> Start Command: " + command.getName()));
    CommandScheduler.getInstance().onCommandInterrupt(command -> log("----X Interrupt Command: " + command.getName()));
    CommandScheduler.getInstance().onCommandFinish(command -> log("----< Finish Command: " + command.getName()));

    SmartDashboard.putBoolean("Robot/Errors/HasError", false);
  }

  public static void start() {
    log("********** Robot Started (version: " + Robot.class.getPackage().getImplementationVersion() + ") **********");
  }

  public static void log(String message) {
    DataLogManager.log("[" + Timer.getFPGATimestamp() + "] " + message);
  }

  public static void log(RobotMode mode) {
    log(">>>>>>>>>> Robot Mode Changed: " + mode + " <<<<<<<<<<");
  }

  public static void debug(String message) {
    log("@@@@@@@@@@ DEBUG: " + message + " @@@@@@@@@@");
  }

  public static void error(String message) {
    log("!!!!!!!!!! ERROR: " + message + " !!!!!!!!!!");
    SmartDashboard.putBoolean("Robot/Error/HasError", true);
    SmartDashboard.putString("Robot/Error/LastError", message);
  }
}
