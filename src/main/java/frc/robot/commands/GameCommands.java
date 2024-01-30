package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class GameCommands {
  private GyroSensor m_gyroSensor;    
  private DriveSubsystem m_driveSubsystem;
  private PoseSubsystem m_poseSubsystem;

  public GameCommands(GyroSensor gyroSensor, DriveSubsystem driveSubsystem, PoseSubsystem poseSubsystem) {
    m_gyroSensor = gyroSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
  }

  public Command alignRobotToSpeaker() {
    Pose3d speaker = Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueSpeaker : Constants.Game.Field.Targets.kRedSpeaker;
    return Commands.sequence(
      m_driveSubsystem.alignToTargetPose(speaker.toPose2d(), m_poseSubsystem::getPose)
    )
    .withName("AlignRobotToSpeaker");
  }

  public Command alignRobotToAmp() {
    Pose3d amp = Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueAmp : Constants.Game.Field.Targets.kRedAmp;
    return Commands.sequence(
      m_driveSubsystem.alignToTargetPose(amp.toPose2d(), m_poseSubsystem::getPose)
    )
    .withName("AlignRobotToAmp");
  } 
}
