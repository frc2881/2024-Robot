package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
  private final GameCommands m_gameCommmands; 
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor; 
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;

  public AutoCommands(
    GameCommands gameCommands,
    BeamBreakSensor launcherBottomBeamBreakSensor,
    BeamBreakSensor launcherTopBeamBreakSensor,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherArmSubsystem launcherArmSubsystem,
    LauncherRollerSubsystem launcherRollerSubsystem
  ) {
    m_gameCommmands = gameCommands; 
    m_launcherBottomBeamBreakSensor = launcherBottomBeamBreakSensor;
    m_launcherTopBeamBreakSensor = launcherTopBeamBreakSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherArmSubsystem = launcherArmSubsystem;
    m_launcherRollerSubsystem = launcherRollerSubsystem;
  }

  // TODO: migate all note pickup paths to use pathFindThenFollowPath methods and only use pathFindToPose for initial move out and preload scoring
  // TODO: move all beam break sensor until predicate checks into the runLaunchAutoCommand function to simplify the commands below
  // TODO: isolate various path following commands into separate commands that can be "warmed up" with the new PathPlannerLib scheduling feature for optimization
  // TODO: create a new shared parallel command for aligning the robot and launcher to target to be reused across all auto commands

  public Command backupScorePickup1() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload1Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote1Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupScorePickup1");
  } 

  public Command backupScorePickup2() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload2Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupScorePickup2");
  } 

  public Command backupScorePickup3() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload3Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand(),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote3Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupScorePickup3");
  } 

  public Command backupScorePickup14() {
    PathPlannerPath pathToPickup4 = PathPlannerPath.fromPathFile("Pickup4");
    PathPlannerPath pathToScoreStage = PathPlannerPath.fromPathFile("ScoreStage");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload1Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        //new WaitCommand(1.0),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote1Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), 
        //new WaitCommand(1.0),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup4, Constants.Drive.kPathFindingConstraints),
          //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote4Poses.notePickupPose()) // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        //new WaitCommand(0.5),
        //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote4Poses.noteScorePose()),
        AutoBuilder.pathfindThenFollowPath(pathToScoreStage, Constants.Drive.kPathFindingConstraints),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
        // new WaitCommand(2.0)
      )
    )
    .withName("BackupShootPickup14");
  } 

  public Command backupScorePickup15() {
    PathPlannerPath pathToPickup5 = PathPlannerPath.fromPathFile("Pickup5");
    PathPlannerPath pathToScoreStage = PathPlannerPath.fromPathFile("ScoreStage");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload1Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote1Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), 
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup5, Constants.Drive.kPathFindingConstraints),
          //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote5Poses.notePickupPose()), // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        AutoBuilder.pathfindThenFollowPath(pathToScoreStage, Constants.Drive.kPathFindingConstraints),
        //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote5Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupShootPickup15");
  } 

  public Command backupScorePickup24() {
    PathPlannerPath pathToPickup4 = PathPlannerPath.fromPathFile("Pickup4");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload2Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        //new WaitCommand(1.0),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), 
        //new WaitCommand(1.0),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup4, Constants.Drive.kPathFindingConstraints),
          //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote4Poses.notePickupPose()) // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        //new WaitCommand(0.5),
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote4Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
        // new WaitCommand(2.0)
      )
    )
    .withName("BackupShootPickup24");
  } 

  public Command backupScorePickup25() {
    PathPlannerPath pathToPickup5 = PathPlannerPath.fromPathFile("Pickup5");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload2Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), 
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup5, Constants.Drive.kPathFindingConstraints),
          //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote5Poses.notePickupPose()), // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote5Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupShootPickup25");
  } 

  public Command backupScorePickup26() {
    PathPlannerPath pathToPickup6 = PathPlannerPath.fromPathFile("Pickup6");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload2Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), 
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup6, Constants.Drive.kPathFindingConstraints),
          //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote6Poses.notePickupPose()), // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote6Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupShootPickup25");
  } 

  public Command backupScorePickup38() {
    PathPlannerPath pathToPickup8 = PathPlannerPath.fromPathFile("Pickup8");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload3Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote3Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand(),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()), 
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup8, Constants.Drive.kPathFindingConstraints),
          //pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote6Poses.notePickupPose()), // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote8Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
    )
    .withName("BackupShootPickup25");
  } 

  public Command scorePickup1() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup1");
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
        m_gameCommmands.runIntakeAutoCommand()
      ),
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0),
        m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionMidRange)
      ),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
    )
    .withName("ScorePickup1");
  } 

  public Command scorePickup2() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup2");
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
        m_gameCommmands.runIntakeAutoCommand()
      )
      .withTimeout(5.0),
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0),
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) 
    )
    .withName("ScorePickup2");
  } 

  public Command scorePickup3() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup3");
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
        m_gameCommmands.runIntakeAutoCommand()
      )
      .withTimeout(5.0),
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand().withTimeout(2.0),
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) 

    )
    .withName("ScorePickup3");
  } 

  public Command scoreMoveout3() {
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      pathFindToNotePose(new Pose2d(2.75, 2.75, Rotation2d.fromDegrees(0)))
    )
    .withName("ScoreMoveout3");
  } 

  public Command scoreSubwooferAuto() {
    return m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionSubwoofer)
    .withTimeout(1.0)
    .andThen(
      m_gameCommmands.runLauncherAutoCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
    )
    .withName("ScoreSubwooferAuto");
  }

  private Command pathFindToNotePose(Pose2d notePose) {
    return Commands.either(
      AutoBuilder.pathfindToPose(notePose, Constants.Drive.kPathFindingConstraints),
      AutoBuilder.pathfindToPoseFlipped(notePose, Constants.Drive.kPathFindingConstraints),
      () -> Robot.getAlliance() == Alliance.Blue
    )
    .withName("PathFindToNotePickup");
  }
}
