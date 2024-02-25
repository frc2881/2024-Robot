package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.LauncherRollerSubsystem.RollerSpeeds;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
  private final GameCommands m_gameCommmands;
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

  public AutoCommands(
    GameCommands gameCommands,
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
    m_gameCommmands = gameCommands; 
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

  // TODO: add reset gyro command as initial sequence command to all autos
  // TODO: update local path constraints instances to use constant 
  // TODO: update local defined auto waypoint poses to use waypoints in constants
  // TODO: consider running intake for launch directly in place of the runAutoLauncherCommand
  // TODO: reconsider why alignLauncherToPositionAutoCommand is stopping 0.5 short of reference target
  // TODO: reconsider why launcher starts at slower speed when aligning before scoring
  // TODO: reconsider the need for manually stopping the rollers now that they run as top-level parallel in auto commands AND they are reset at the end of auto / beginning of teleop

  private Command resetGyroCommand() { 
    return Commands
    .runOnce(() -> m_gyroSensor.reset(m_poseSubsystem.getPose().getRotation().getDegrees()))
    .withName("ResetGyro"); 
  }

  public Command shootPickup1() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup1");
    PathConstraints constraints = new PathConstraints(3, 3, 540.00, 720.00);
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, constraints),
        m_gameCommmands.runIntakeCommand(IntakeLocation.Front)
      ),
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand(),
        m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionMidRange)
      ),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
      
      //AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.10, 7.00, Rotation2d.fromDegrees(0)), constraints)
      //AutoBuilder.followPath(path1)
    )
    .withName("Test");
  } 

  public Command backupShootPickup1() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("BackupPickup1");
    PathConstraints constraints = new PathConstraints(3, 3, 540.00, 720.00);
    return Commands
    .sequence(
      Commands.parallel(
        Commands.either(
          AutoBuilder.pathfindToPose(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints),
          AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints),
          () -> Robot.getAlliance() == Alliance.Blue
        ),
        m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionBlueLine)
      ),
      m_gameCommmands.alignRobotToTargetCommand(),
      m_gameCommmands.runLauncherAutoCommand(0.0, false)
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, constraints),
        m_gameCommmands.runIntakeCommand(IntakeLocation.Front)
      ),
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand(),
        m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionMidRange)
      ),
      m_gameCommmands.runLauncherAutoCommand(0.0, true)
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
    )
    .finallyDo(() -> m_gameCommmands.stopLauncherRollersCommand())
    .withName("Test");
  } 

  public Command backupShootPickup14() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("BackupPickup1");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("BackupPickup4");
    PathConstraints constraints = new PathConstraints(3, 3, 540.00, 720.00);
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new RollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        Commands.parallel(
          Commands.either(
            AutoBuilder.pathfindToPose(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints),
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints), // Go to first position
            () -> Robot.getAlliance() == Alliance.Blue
          ),
          m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionBlueLine) // move arm to blue line position
        ),
        m_gameCommmands.alignRobotToTargetCommand(), // Align robot
        m_gameCommmands.runLauncherAutoCommand(0.0, false) // SHOOT FIRST NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(path1, constraints), // grab next note
          m_gameCommmands.runIntakeCommand(IntakeLocation.Front) // grab next note
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(), // align to speaker
          m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionMidRange) // align launcher to speaker
        ),
        m_gameCommmands.runLauncherAutoCommand(0.0, false) // SHOOT SECOND NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), // TODO: Create run launcher command that stops when note shot
        
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(path2, constraints), // grab next note
          m_gameCommmands.runIntakeCommand(IntakeLocation.Front) // grab next note
        ),
        Commands.parallel(
          Commands.either(
            AutoBuilder.pathfindToPose(new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)), constraints),
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)), constraints),
            () -> Robot.getAlliance() == Alliance.Blue
          ),
          m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionLongRange)
        ),
        m_gameCommmands.alignRobotToTargetCommand(),
        m_gameCommmands.runLauncherAutoCommand(0.0, false) // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
      )
    )
    .finallyDo(() -> m_gameCommmands.stopLauncherRollersCommand())
    .withName("BackupShootPickup14");
  } 

  public Command shootPickup2() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup2");
    PathConstraints constraints = new PathConstraints(1.5, 1.5, 540.00, 720.00);
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, constraints),
        m_gameCommmands.runIntakeCommand(IntakeLocation.Front)
      )
      .withTimeout(5.0), // TODO: make it so it ends when intake is done (race???)
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand(), // TODO: Make it so it ends when at position, not when timeouts over
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
      
      //AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.10, 7.00, Rotation2d.fromDegrees(0)), constraints)
      //AutoBuilder.followPath(path1)
    )
    .withName("Test");
  } 

  public Command shootPickup3() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup3");
    PathConstraints constraints = new PathConstraints(1.5, 1.5, 540.00, 720.00);
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, constraints),
        m_gameCommmands.runIntakeCommand(IntakeLocation.Front)
      )
      .withTimeout(5.0), // TODO: make it so it ends when intake is done (race???)
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand(), // TODO: Make it so it ends when at position, not when timeouts over
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
      
      //AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.10, 7.00, Rotation2d.fromDegrees(0)), constraints)
      //AutoBuilder.followPath(path1)
    )
    .withName("Test");
  } 

  public Command scoreSubwooferAuto() {
    return m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionSubwoofer, true)
      .withTimeout(1.0)
    .andThen(
      m_gameCommmands.runLauncherCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
    )
    .withName("ScoreSubwooferAuto");
  }

  public Command Pos1Note1Path() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Position1Grab");
    return Commands
    .sequence(
      AutoBuilder.followPath(path1)
    )
    .withName("Pos1To3NotePath");
  }

  public Command Pos2Note1Path() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Position2Grab");
    return Commands
    .sequence(
      AutoBuilder.followPath(path1)
    )
    .withName("Pos1To3NotePath");
  }

  public Command Pos3Note1Path() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Position3Grab");
    return Commands
    .sequence(
      AutoBuilder.followPath(path1)
    )
    .withName("Pos1To3NotePath");
  }

}
