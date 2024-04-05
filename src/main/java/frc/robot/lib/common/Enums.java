package frc.robot.lib.common;

public class Enums {
  public static enum RobotMode { Disabled, Auto, Teleop, Test, Sim; }
  public static enum RobotState { Disabled, Enabled, EStopped; }
  public static enum MotorDirection { None, Forward, Reverse; }
  public static enum SwerveModuleLocation { FrontLeft, FrontRight, RearLeft, RearRight; }
  public static enum DriveOrientation { Field, Robot; }
  public static enum DriveSpeedMode { Competition, Training; }
  public static enum DriveLockState { Unlocked, Locked; }
  public static enum DriveDriftCorrection { Enabled, Disabled; }

  public static enum LightsMode { 
    Default, 
    IntakeReady, 
    IntakeNotReady, 
    LaunchReady,
    VisionNotReady;
  }
  
  public static enum AutoPath {
    ScorePreload1,
    ScorePreload2,
    ScorePreload3,
    Pickup1,
    Pickup13,
    Pickup2,
    Pickup21,
    Pickup23,
    Pickup3,
    Pickup31,
    Pickup4,
    Pickup5,
    Pickup61,
    Pickup62,
    Pickup63,
    Pickup72,
    Pickup73,
    Pickup8,
    ScoreStage1,
    ScoreStage2,
    ScoreStage3;
  }
}
