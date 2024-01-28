package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
    
    private DriveSubsystem m_driveSubsystem;
    private PoseSubsystem m_poseSubsystem;
    private GyroSensor m_gyro;

    public GameCommands(DriveSubsystem driveSubsystem, PoseSubsystem poseSubsystem, GyroSensor gyro) {
      m_driveSubsystem = driveSubsystem;
      m_poseSubsystem = poseSubsystem;
      m_gyro = gyro;
    }

    /// LAUNCHER
    public Command alignRobotToSpeaker() {
      Pose3d speaker = Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueSpeaker : Constants.Game.Field.Targets.kRedSpeaker;
      return Commands.sequence(
        m_driveSubsystem.alignToTargetPose(new Pose2d(speaker.getX(), speaker.getY(), speaker.getRotation().toRotation2d()), m_poseSubsystem::getPose)
      )
      .withName("alignRobotToSpeaker");
    }

    public Command alignRobotToAmp() {
      Pose3d amp = Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueAmp : Constants.Game.Field.Targets.kRedAmp;
      return Commands.sequence(
        m_driveSubsystem.alignToTargetPose(new Pose2d(amp.getX(), amp.getY(), amp.getRotation().toRotation2d()), m_poseSubsystem::getPose)
      )
      .withName("alignRobotToAmp");
    }

    
}
