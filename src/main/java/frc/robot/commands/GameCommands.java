package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.lib.controllers.GameController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class GameCommands {  
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor; 
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final GameController m_driverController;
  private final GameController m_operatorControlller;
  
  public GameCommands( 
    BeamBreakSensor launcherBottomBeamBreakSensor,
    BeamBreakSensor launcherTopBeamBreakSensor,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherArmSubsystem launcherArmSubsystem,
    LauncherRollerSubsystem launcherRollerSubsystem,
    ClimberSubsystem climberSubsystem,
    GameController driverController,
    GameController operatorControlller
  ) {
    m_launcherBottomBeamBreakSensor = launcherBottomBeamBreakSensor;
    m_launcherTopBeamBreakSensor = launcherTopBeamBreakSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherArmSubsystem = launcherArmSubsystem;
    m_launcherRollerSubsystem = launcherRollerSubsystem;
    m_climberSubsystem = climberSubsystem;
    m_driverController = driverController;
    m_operatorControlller = operatorControlller;
  }

  public Command runIntakeCommand() {
    return 
    m_intakeSubsystem.runIntakeCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    .deadlineWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionIntake))
    .andThen(
      new WaitCommand(0.05),
      m_intakeSubsystem.runAdjustNotePositionCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    )
    .andThen(rumbleControllersCommand(true, false))
    .withName("RunIntake");
  }

  public Command runIntakeAutoCommand() {
    return 
    m_intakeSubsystem.runIntakeAutoCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    .deadlineWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionIntake))
    .andThen(
      new WaitCommand(0.05),
      m_intakeSubsystem.runAdjustNotePositionCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    )
    .withName("RunIntakeAuto");
  }

  public Command runEjectCommand() {
    return
    m_intakeSubsystem.runEjectCommand()
    .withName("RunEject");
  }

  public Command runReloadCommand() {
    return Commands.sequence(
      m_intakeSubsystem.runEjectCommand().withTimeout(0.23),
      m_intakeSubsystem.runIntakeCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    )
    .withName("RunReload");
  }

  public Command alignRobotToTargetCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, m_poseSubsystem::getTargetYaw)
    .andThen(rumbleControllersCommand(true, true))
    .withName("AlignRobotToTarget");
  }

  public Command alignRobotToTargetAutoCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, m_poseSubsystem::getTargetYaw)
    .withTimeout(2.0)
    .withName("AlignRobotToTargetAuto");
  }
  
  public Command alignLauncherToTargetCommand(boolean isRollersEnabled) {
    return
    m_launcherArmSubsystem.alignToTargetCommand(m_poseSubsystem::getTargetDistance)
    .alongWith(
      m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kWarmupLauncherSpeeds)
      .onlyIf(() -> isRollersEnabled)
    )
    .withName("AlignLauncherToTarget");
  }

  public Command alignLauncherToTargetAutoCommand() {
    return
    m_launcherArmSubsystem.alignToTargetAutoCommand(m_poseSubsystem::getTargetDistance)
    .withTimeout(2.0)
    .withName("AlignLauncherToTargetAuto");
  }

  public Command alignLauncherToPositionCommand(double position, boolean isRollersEnabled) {
    return
    m_launcherArmSubsystem.alignToPositionCommand(position)
    .alongWith(
      m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kWarmupLauncherSpeeds)
      .onlyIf(() -> isRollersEnabled)
    )
    .withName("AlignLauncherToPosition");
  }

  public Command alignLauncherToPositionAutoCommand(double position) {
    return
    m_launcherArmSubsystem.alignToPositionAutoCommand(position)
    .withTimeout(2.0)
    .withName("AlignLauncherToPositionAuto");
  }

  public Command alignLauncherToAmpCommand(boolean isRollersEnabled) {
    return
    m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionAmp)
    .alongWith(
      m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kAmpLauncherSpeeds)
      .onlyIf(() -> isRollersEnabled)
    )
    .withName("AlignLauncherToAmp");
  }

  public Command alignLauncherForShuttleCommand() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kShuttleLauncherSpeeds),
      Commands.sequence(
        alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionShuttle)
      )
    )
    .withName("AlignLauncherForShuttle");
  }

  public Command startLauncherRollersAutoCommand() {
    return
    m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kDefaultLauncherSpeeds)
    .withName("StartLauncherRollersAuto");
  }

  public Command runLauncherCommand() {
    return 
    m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kDefaultLauncherSpeeds)
    .alongWith(
      Commands.waitSeconds(0.5)
      .andThen(m_intakeSubsystem.runLaunchCommand())
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .finallyDo(() -> { 
      m_driveSubsystem.clearTargetAlignment();
      m_launcherArmSubsystem.clearTargetAlignment(); 
    })
    .withName("RunLauncher");
  }

  public Command runLauncherAutoCommand() {
    return 
    m_intakeSubsystem.runLaunchCommand()
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
    .finallyDo(() -> { 
      m_driveSubsystem.clearTargetAlignment();
      m_launcherArmSubsystem.clearTargetAlignment(); 
    })
    .withName("RunLauncherAuto");
  }

  public Command runLauncherForAmpCommand() {
    return 
    m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kAmpLauncherSpeeds)
    .alongWith(
      Commands.waitSeconds(0.5)
      .andThen(m_intakeSubsystem.runLaunchCommand())
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .finallyDo(() -> { 
      m_driveSubsystem.clearTargetAlignment();
      m_launcherArmSubsystem.clearTargetAlignment(); 
    })
    .withName("RunLauncherForAmp");
  }

  public Command runLauncherForShuttleCommand() {
    return runLauncherAutoCommand()
    .withName("RunLauncherForShuttle");
  }

  public Command rumbleControllersCommand(boolean isDriverRumbleEnabled, boolean isRumbleOperatorEnabled) {
    return Commands.parallel(
      m_driverController.rumbleShort().onlyIf(() -> isDriverRumbleEnabled),
      m_operatorControlller.rumbleShort().onlyIf(() -> isRumbleOperatorEnabled)
    )
    .withName("RumbleControllers");
  }
}
