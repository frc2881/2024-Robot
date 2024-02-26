package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Enums.IntakeLocation;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem.RollerSpeeds;

public class GameCommands {
  private final GyroSensor m_gyroSensor;   
  private final BeamBreakSensor m_intakeBeamBreakSensor;
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor; 
  private final DistanceSensor m_intakeDistanceSensor;
  private final DistanceSensor m_launcherDistanceSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final LightsController m_lightsController;

  private double m_launchSpeed;

  public GameCommands(
    GyroSensor gyroSensor, 
    BeamBreakSensor intakeBeamBreakSensor,
    BeamBreakSensor launcherBottomBeamBreakSensor,
    BeamBreakSensor launcherTopBeamBreakSensor,
    DistanceSensor intakeDistanceSensor,
    DistanceSensor launcherDistanceSensor,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    FeederSubsystem feederSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherArmSubsystem launcherArmSubsystem,
    LauncherRollerSubsystem launcherRollerSubsystem,
    ClimberSubsystem climberSubsystem,
    LightsController lightsController
  ) {
    m_gyroSensor = gyroSensor;
    m_intakeBeamBreakSensor = intakeBeamBreakSensor;
    m_launcherBottomBeamBreakSensor = launcherBottomBeamBreakSensor;
    m_launcherTopBeamBreakSensor = launcherTopBeamBreakSensor;
    m_intakeDistanceSensor = intakeDistanceSensor;
    m_launcherDistanceSensor = launcherDistanceSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherArmSubsystem = launcherArmSubsystem;
    m_launcherRollerSubsystem = launcherRollerSubsystem;
    m_climberSubsystem = climberSubsystem;
    m_lightsController = lightsController;
  }

  public Command runIntakeCommand(IntakeLocation intakeDirection) {
    switch (intakeDirection) {
      case Front:
        return 
        m_intakeSubsystem.runIntakeFrontCommand(m_intakeBeamBreakSensor::hasTarget, m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
        .raceWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionIntake))
        .withName("RunIntakeFront");
      case Rear:
        return
        m_intakeSubsystem.runIntakeRearCommand(m_intakeBeamBreakSensor::hasTarget, m_launcherTopBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
        .raceWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionIntake))
        .withName("RunIntakeRear");
      default:
        return Commands.none();
    }
  }

  public Command runEjectCommand(IntakeLocation intakeDirection) {
    switch (intakeDirection) {
      case Front:
        return
        m_intakeSubsystem.runEjectFrontCommand()
        .withName("RunEjectFront");
      case Rear:
        return
        m_intakeSubsystem.runEjectRearCommand()
        .withName("RunEjectRear");
      default:
        return Commands.none();
    }
  }

  public Command alignRobotToTargetCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, () -> getTargetPose())
    .withName("AlignRobotToTarget");
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
    m_launcherArmSubsystem.alignToPositionCommand(position)
    .until(() -> Math.abs(m_launcherArmSubsystem.getArmPosition() - position) < 0.1)
    .withName("AlignLauncherToPosition");
  }

  public Command alignLauncherToTargetCommand(boolean isRollersEnabled) {
    return
    m_launcherArmSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, () -> getTargetPose())
    .alongWith(
      m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kWarmupLauncherSpeeds)
      .onlyIf(() -> isRollersEnabled)
    )
    .withName("AlignLauncherToTarget");
  }
  
  public Command runLauncherCommand() {
    return 
    m_launcherRollerSubsystem.runCommand(() -> getLauncherRollerSpeeds())
    .alongWith(
      Commands.waitSeconds(1.5)
      .andThen(m_intakeSubsystem.runLaunchCommand())
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .withName("RunLauncher");
  }

  public Command runLauncherAutoCommand() {
    return 
    Commands.sequence( // TODO: Refactor later
      m_intakeSubsystem.runLaunchCommand()
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .withName("RunLauncher");
  }

  public Command moveToClimbCommand() {
    return 
    m_launcherArmSubsystem.alignToPositionCommand(1.0)
    .alongWith(m_feederSubsystem.moveArmOutCommand().withTimeout(0.5))
    .alongWith(m_climberSubsystem.moveArmToPositionCommand(Constants.Climber.kArmMotorForwardSoftLimit - 0.1))
    .withName("MoveToClimb");
  }

  public Command climbCommand() {
    return 
    m_climberSubsystem.moveArmToPositionCommand(0.0) // TODO: update?
    .withName("Climb");
  }

  public Command resetSubsystems() {
    return 
    m_feederSubsystem.resetCommand()
    .alongWith(m_climberSubsystem.resetCommand())
    .withName("ResetSubsystems");
  }

  private Pose3d getTargetPose() {
    return 
    Robot.getAlliance() == Alliance.Blue 
    ? Constants.Game.Field.Targets.kBlueSpeaker 
    : Constants.Game.Field.Targets.kRedSpeaker;
  }

  private RollerSpeeds getLauncherRollerSpeeds() {
    if (m_launcherArmSubsystem.getArmPosition() >= Constants.Launcher.kArmPositionAmp) {
      return new RollerSpeeds(0.6, 0.6);
    } else {
      return new RollerSpeeds(0.8, 0.8);
    }
  }
}
