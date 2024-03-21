package frc.robot.lib.common;

import edu.wpi.first.math.geometry.Pose2d;

public class Records {

  public record PIDConstants(
    double P, 
    double I, 
    double D, 
    double FF
  ){}

  public record AutoPoses(
    Pose2d notePickupPose, 
    Pose2d noteScorePose
  ){}

  public record LauncherArmPosition(
    double distance,
    double position
  ){}

  public record LauncherRollerSpeeds(
    double top, 
    double bottom
  ){}
}
