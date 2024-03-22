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
    LaunchReady; 
  }
  
  public static enum AutoPose {
    ScorePreload1,
    ScorePreload2,
    ScorePreload3,
    Pickup1,
    Pickup2,
    Pickup3
  }
  
  public static enum AutoPath {
    ScorePreload1,
    ScorePreload2,
    ScorePreload3,
    Pickup1,
    Pickup2,
    Pickup3,
    Pickup4,
    Pickup5,
    Pickup6,
    Pickup7,
    Pickup8,
    ScoreStage1,
    ScoreStage2,
    ScoreStage3,
    MoveOut1,
    MoveOut2,
    MoveOut3;
  }
}
