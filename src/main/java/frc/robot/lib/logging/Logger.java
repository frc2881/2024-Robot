package frc.robot.lib.logging;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

  public static void log(REVLibError error, String source) {
    Timer.delay(0.001);
    if (error != REVLibError.kOk) {
      log("!!!!!!!!!! REVLibError Returned: " + source + ":" + error.toString() + " !!!!!!!!!!");
    }
  }

  public static void debug(String message) {
    log("@@@@@@@@@@ DEBUG: " + message + " @@@@@@@@@@");
  }
}
