package frc.robot.lib.common;

public class Records {
  public record PIDConstants(double P, double I, double D, double FF) {}

  public record LauncherArmPosition(double distance, double position) {}
  
  public record LauncherRollerSpeeds(double top, double bottom) {}
}
