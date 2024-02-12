package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class GameCommands {
  private final GyroSensor m_gyroSensor;    
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

  public GameCommands(
    GyroSensor gyroSensor, 
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

  // TODO: Wait to move the note into launcher until launcher is in default position
  public Command runFrontIntakeCommand() {
    return 
    m_intakeSubsystem.runIntakeFromFrontCommand(m_intakeDistanceSensor::hasTarget, m_launcherDistanceSensor::hasTarget)
    .alongWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kDefaultPosition))  
    .andThen(getNoteIntoLaunchPositionCommand(m_launcherDistanceSensor::getDistance)).withTimeout(5.0)
    .withName("RunFrontIntakeCommand");

  }

  // TODO: this needs testing and tuning with motor speed, sensor distance/note detection
  public Command runRearIntakeCommand() {
    return
    m_intakeSubsystem.runIntakeFromRearCommand(m_intakeDistanceSensor::hasTarget, m_launcherDistanceSensor::hasTarget)
    .alongWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kDefaultPosition))
    // .andThen(getNoteIntoLaunchPositionCommand(m_launcherDistanceSensor::getDistance)).withTimeout(5.0)
    .withName("RunRearIntakeCommand");
  }

  // TODO: this needs testing to confirm proper ejection of note out of bottom of robot
  public Command runEjectIntakeCommand() {
    return
    m_intakeSubsystem.runIntakeEjectCommand()
    .withName("RunEjectIntakeCommand");
  }

  public Command getNoteIntoLaunchPositionCommand(Supplier<Double> distanceSupplier){
    return Commands.repeatingSequence(
      m_intakeSubsystem.runIntakeForNotePositionCommand().withTimeout(0.1) // Might need to slow down intake0.2)
    )
    .until(() -> distanceSupplier.get() > 3.5)
    .withName("getNoteIntoLaunchPosition " + distanceSupplier.get().toString());
  }

  // TODO: build launch sequence command - see IntakeSubsystem - runFrontIntakeCommand for example of sequence and conditional logic (assumes that robot has already been aligned by driver/rotation and operator/elevation)
  // TODO: after testing basic launcher command, implement separate commands for launching into speaker vs. amp with different launcher roller speed configurations (see inside LauncherSubsystem for TODO)
  public Command runLauncherCommand() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runRollersCommand(-0.8, 0.8),
      //m_launcherArmSubsystem.alignToPositionCommand(m_launcherArmSubsystem.getPosition()),
      Commands.sequence(
        new WaitCommand(1.5),
        m_intakeSubsystem.runIntakeForLaunchCommand()
        )
    )
    .unless(() -> !m_launcherDistanceSensor.hasTarget())
    .withName("RunLauncher");
    
    
    // - add reasonable wait (1 second) and then check if launcher distance sensor no longer has target (note has launched)
    // - add reasonable timeout to end command which will stop launcher rollers and intake belts
  }

  public Command tiltLauncherCommand (Supplier<Double> speed) {
    return m_launcherArmSubsystem.tiltLauncherCommand(speed);
  }

  public Command moveArmCommand (Supplier<Double> speed) {
    return m_climberSubsystem.moveArmCommand(speed);
  }

  public Command alignLauncherCommand() {
    return m_launcherArmSubsystem.alignLauncherCommand(() -> m_launcherDistanceSensor.hasTarget());
  }

  // TODO: this needs testing once vision cameras are mounted and configured
  public Command alignRobotToSpeakerCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getSpeaker().toPose2d())
    .withName("AlignRobotToSpeaker");
  }

  // TODO: this needs testing once vision cameras are mounted and configured
  public Command alignLauncherToSpeakerCommand() {
    return
    m_launcherArmSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getSpeaker())
    .withName("AlignLauncherToSpeaker");
  }

  public Command resetSubsystems() {
    return 
    m_feederSubsystem.resetCommand()
    .alongWith(
      m_launcherArmSubsystem.resetCommand(),
      m_climberSubsystem.resetCommand()
    )
    .withName("ResetSubsystems");
  }

  private static Pose3d getSpeaker() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueSpeaker : Constants.Game.Field.Targets.kRedSpeaker;
  }

}
