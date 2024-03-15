package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.common.Utils;
import frc.robot.lib.controllers.GameController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class GameCommands {
  private final GyroSensor m_gyroSensor;   
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
    GyroSensor gyroSensor, 
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
    m_gyroSensor = gyroSensor;
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

  // TODO: take out intake direction
  public Command runIntakeCommand() {
      return 
      m_intakeSubsystem.runIntakeFrontCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
      .raceWith(m_launcherArmSubsystem.alignToIntakePositionCommand())
      .andThen(
        new WaitCommand(0.05),
        m_intakeSubsystem.adjustNotePositionCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
      )
      .withName("RunIntakeFront");
  }

  public Command runIntakeAutoCommand() {
    return 
    m_intakeSubsystem.runIntakeFrontAutoCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    .raceWith(m_launcherArmSubsystem.alignToIntakePositionCommand())
    .andThen(
      new WaitCommand(0.05),
      m_intakeSubsystem.adjustNotePositionCommand(m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    )
    .withName("RunIntakeFront");
}

  public Command runEjectCommand() {
      return
      m_intakeSubsystem.runEjectFrontCommand()
      .withName("RunEjectFront");
  }

  public Command alignRobotToTargetCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, m_poseSubsystem::getTargetYaw)
    .withName("AlignRobotToTarget");
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
    .withName("AlignLauncherToTarget");
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

  public Command alignLauncherToAmpCommand(boolean isRollersEnabled) {
    return
    m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionAmp)
    .alongWith(
      m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kAmpLauncherSpeeds)
      .onlyIf(() -> isRollersEnabled)
    )
    .withName("AlignLauncherToAmp");
  }

  public Command alignLauncherToPositionAutoCommand(double position) {
    return
    m_launcherArmSubsystem.alignToPositionCommand(position)
    .until(() -> Math.abs(m_launcherArmSubsystem.getArmPosition() - position) < 0.1)
    .withName("AlignLauncherToPositionAuto");
  }

  public Command runLauncherCommand() {
    return 
    m_launcherRollerSubsystem.runCommand(() -> m_launcherRollerSubsystem.getSpeedsForArmPosition(m_poseSubsystem::getTargetDistance))
    .alongWith(
      Commands.waitSeconds(0.5)
      .andThen(m_intakeSubsystem.runLaunchCommand())
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .withName("RunLauncher");
  }

  public Command runLauncherAmpCommand() {
    return 
    m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kAmpLauncherSpeeds)
    .alongWith(
      Commands.waitSeconds(0.5)
      .andThen(m_intakeSubsystem.runLaunchCommand())
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .withName("RunLauncher");
  }

  public Command runLauncherAutoCommand() {
    return 
    m_intakeSubsystem.runLaunchCommand()
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .withName("RunLauncherAuto");
  }

  public Command shuttleCommand() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.60, 0.60)),
      Commands.sequence(
        alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionShuttle)
      )
    )
    .withName("shuttleNote");
  }

  public Command shootShuttleCommand() {
    return runLauncherAutoCommand();
  }

  public Command moveToClimbCommand() {
    return 
    m_climberSubsystem.moveArmToPositionCommand(Constants.Climber.kArmMotorForwardSoftLimit)
    .withName("MoveToClimb");
  }

  public Command climbCommand() {
    return 
    m_climberSubsystem.moveArmToPositionCommand(0.0)
    .withName("Climb");
  }

  public Command rumbleControllers(boolean rumbleDriver, boolean rumbleOperator) {
    return Commands.parallel(
      m_driverController.rumbleShort().onlyIf(() -> rumbleDriver),
      m_operatorControlller.rumbleShort().onlyIf(() -> rumbleOperator)
    )
    .withName("RumbleControllers");
  }

  public Command resetGyroToPoseCommand() { 
    return Commands
    .runOnce(() -> m_gyroSensor.reset(Utils.wrapAngle(m_poseSubsystem.getPose().getRotation().getDegrees())))
    .withName("ResetGyro"); 
  }

  public Command resetSubsystems() {
    return 
    m_climberSubsystem.resetCommand()
    .withName("ResetSubsystems");
  }
}
