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

  // TODO: this needs testing and tuning with motor speed, sensor distance/note detection
  public Command runFrontIntakeCommand() {
    return 
    m_intakeSubsystem.runIntakeFromFrontCommand(m_intakeDistanceSensor::hasTarget, m_launcherDistanceSensor::hasTarget)
    .withName("RunFrontIntakeCommand");
  }

  // TODO: this needs testing and tuning with motor speed, sensor distance/note detection
  public Command runRearIntakeCommand() {
    return
    m_intakeSubsystem.runIntakeFromRearCommand(m_intakeDistanceSensor::hasTarget, m_launcherDistanceSensor::hasTarget)
    .withName("RunRearIntakeCommand");
  }

  // TODO: this needs testing to confirm proper ejection of note out of bottom of robot
  public Command runEjectIntakeCommand() {
    return
    m_intakeSubsystem.runIntakeEjectCommand()
    .withName("RunEjectIntakeCommand");
  }

  // TODO: build launch sequence command - see IntakeSubsystem - runFrontIntakeCommand for example of sequence and conditional logic (assumes that robot has already been aligned by driver/rotation and operator/elevation)
  // TODO: after testing basic launcher command, implement separate commands for launching into speaker vs. amp with different launcher roller speed configurations (see inside LauncherSubsystem for TODO)
  public Command runLauncherCommand() {
    return Commands.none();
    // - check if launcher distance sensor has target (note is present and ready to launch)
    // - in parallel: run launcher rollers, add wait for launcher roller spin up ~ 1 second or less, run intake subsystem launch command to push note into rollers
    // - add reasonable wait (1 second) and then check if launcher distance sensor no longer has target (note has launched)
    // - add reasonable timeout to end command which will stop launcher rollers and intake belts
  }

  // TODO: this needs testing once vision cameras are mounted and configured
  public Command alignRobotToSpeakerCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getSpeaker().toPose2d())
    .withName("AlignRobotToSpeaker");
  }

  // TODO: this needs testing once vision cameras are mounted and configured
  public Command alignRobotToAmpCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getAmp().toPose2d())
    .withName("AlignRobotToAmp");
  } 

  // TODO: this needs testing once vision cameras are mounted and configured
  public Command alignLauncherToSpeakerCommand() {
    return
    m_launcherSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getSpeaker())
    .withName("AlignLauncherToSpeaker");
  }

  // TODO: this needs testing once vision cameras are mounted and configured
  public Command alignLauncherToAmpCommand() {
    return
    m_launcherSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getAmp())
    .withName("AlignLauncherToAmp");
  }

  private static Pose3d getSpeaker() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueSpeaker : Constants.Game.Field.Targets.kRedSpeaker;
  }

  private static Pose3d getAmp() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueAmp : Constants.Game.Field.Targets.kRedAmp;
  }
}
