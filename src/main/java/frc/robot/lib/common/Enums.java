package frc.robot.lib.common;

public class Enums {
  public static enum RobotMode { Disabled, Auto, Teleop, Test, Sim; }
  public static enum RobotState { Disabled, Enabled, EStopped; }
  public static enum MotorDirection { None, Forward, Reverse; }
  public static enum DriveOrientation { Field, Robot; }
  public static enum DriveSpeedMode { Competition, Training; }
  public static enum DriveLockState { Unlocked, Locked; }
  public static enum DriveDriftCorrection { Enabled, Disabled; }
  public static enum SwerveModuleLocation { FrontLeft, FrontRight, RearLeft, RearRight; }
  public static enum LightsMode { Default, IntakeReady, LaunchReady; }
}
