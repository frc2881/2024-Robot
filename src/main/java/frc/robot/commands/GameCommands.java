package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class GameCommands {
  private final GyroSensor m_gyroSensor;    
  private final DistanceSensor m_intakeDistanceSensor;
  private final DistanceSensor m_launcherDistanceSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final ArmSubsystem m_armSubsystem;
  private final LightsController m_lightsController;

  public GameCommands(
    GyroSensor gyroSensor, 
    DistanceSensor intakeDistanceSensor,
    DistanceSensor launcherDistanceSensor,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    FeederSubsystem feederSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherSubsystem launcherSubsystem,
    ArmSubsystem armSubsystem,
    LightsController lightsController
  ) {
    m_gyroSensor = gyroSensor;
    m_intakeDistanceSensor = intakeDistanceSensor;
    m_launcherDistanceSensor = launcherDistanceSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherSubsystem = launcherSubsystem;
    m_armSubsystem = armSubsystem;
    m_lightsController = lightsController;
  }

  // TODO: build command to run intake belts from driver controller triggers and set direction of belts and rollers based on intake/launcher sensor hasTarget booleans
  // TODO: build command to run launcher and intake rollers from operator controller trigger with launcher sensor hasTarget boolean and timeout to complete

  public Command alignRobotToSpeakerCommand() {
    return Commands
      .sequence(
        m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getSpeaker().toPose2d())
      )
      .withName("AlignRobotToSpeaker");
  }

  public Command alignRobotToAmpCommand() {
    return Commands
      .sequence(
        m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getAmp().toPose2d())
      )
      .withName("AlignRobotToAmp");
  } 

  public Command alignLauncherToSpeakerCommand() {
    return Commands
      .sequence(
        m_launcherSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getSpeaker())
      )
      .withName("AlignLauncherToSpeaker");
  }

  public Command alignLauncherToAmpCommand() {
    return Commands
      .sequence(
        m_launcherSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getAmp())
      )
      .withName("AlignLauncherToAmp");
  }

  private static Pose3d getSpeaker() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueSpeaker : Constants.Game.Field.Targets.kRedSpeaker;
  }

  private static Pose3d getAmp() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueAmp : Constants.Game.Field.Targets.kRedAmp;
  }
}
