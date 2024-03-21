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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
  private final GameCommands m_gameCommmands; 
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;

  public AutoCommands(
    GameCommands gameCommands,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherArmSubsystem launcherArmSubsystem,
    LauncherRollerSubsystem launcherRollerSubsystem
  ) {
    m_gameCommmands = gameCommands; 
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherArmSubsystem = launcherArmSubsystem;
    m_launcherRollerSubsystem = launcherRollerSubsystem;
  }

  // TODO: migate all note pickup paths to use pathFindThenFollowPath methods and only use pathFindToPose for initial move out and preload scoring
  // TODO: move the run intake command to use race/deadline for running a path in parallel where if the intake successfully gets a note, the path can immediately stop and transition into alignment/scoring (no need waste time completing the path)
  // TODO: construct all commands to use race/deadline with the full auto sequence as first command and parallel run of launcher rollers (which will end at the completion of the sequence)
  // TODO: add timeouts to note pickup/scoring sequences that allows to move on to next pickup/score option if the note pickup is missed

  private Command pathFindToPose(Pose2d pose) {
    return 
    Commands.either(
      AutoBuilder.pathfindToPose(pose, Constants.Drive.kPathFindingConstraints),
      AutoBuilder.pathfindToPoseFlipped(pose, Constants.Drive.kPathFindingConstraints),
      () -> Robot.getAlliance() == Alliance.Blue
    )
    .withName("PathFindToPose");
  }

  private Command pickup(PathPlannerPath path) {
    return
    m_gameCommmands.runIntakeAutoCommand()
    .deadlineWith(AutoBuilder.pathfindThenFollowPath(path, Constants.Drive.kPathFindingConstraints))
    .withName("PickupWithPath");
  }

  private Command pickup(Pose2d pose) {
    return
    m_gameCommmands.runIntakeAutoCommand()
    .deadlineWith(pathFindToPose(pose))
    .withName("PickupWithPose");
  }

  private Command move(PathPlannerPath path) {
    return
    AutoBuilder.pathfindThenFollowPath(path, Constants.Drive.kPathFindingConstraints)
    .withName("MoveWithPath");
  }

  private Command move(Pose2d pose) {
    return
    pathFindToPose(pose)
    .withName("MoveWithPose");
  }

  private Command run() {
    return
    m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kDefaultLauncherSpeeds);
  }

  private Command launch() {
    return
    m_gameCommmands.alignRobotToTargetCommand()
    .alongWith(m_gameCommmands.alignLauncherToTargetAutoCommand())
    .withTimeout(2.0)
    .andThen(m_gameCommmands.runLauncherAutoCommand())
    .withName("AlignToTargetAndLaunch");
  }

  // ============================================================

  public Command backupScorePickup1() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload1")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote1Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch()
      )
    )
    .withName("BackupScorePickup1");
  } 

  public Command backupScorePickup2() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload2")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch()
      )
    )
    .withName("BackupScorePickup2");
  } 

  public Command backupScorePickup3() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload3")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote3Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch()
      )
    )
    .withName("BackupScorePickup3");
  } 

  public Command auto_1_0_1_4() {
    return 
    Commands.sequence(
      move(Constants.Game.Auto.kNoteScoringPoses.get("Preload1")),
      launch(),
      pickup(Constants.Game.Auto.Waypoints.kNote1Poses.notePickupPose()),
      launch(), 
      pickup(Constants.Game.Auto.kNotePickupPaths.get("Pickup4")),
      move(Constants.Game.Auto.kNoteScoringPaths.get("ScoreStage1")),
      launch()
    )
    .deadlineWith(run())
    .withName("Auto_1_0_1_4");
  } 

  public Command backupScorePickup15() {
    PathPlannerPath pathToPickup5 = PathPlannerPath.fromPathFile("Pickup5");
    PathPlannerPath pathToScoreStage = PathPlannerPath.fromPathFile("ScoreStage1");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload1")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote1Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch(), 
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup5, Constants.Drive.kPathFindingConstraints),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        AutoBuilder.pathfindThenFollowPath(pathToScoreStage, Constants.Drive.kPathFindingConstraints),
        launch()
      )
    )
    .withName("BackupShootPickup15");
  } 

  public Command backupScorePickup24() {
    PathPlannerPath pathToPickup4 = PathPlannerPath.fromPathFile("Pickup4");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload2")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch(),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup4, Constants.Drive.kPathFindingConstraints),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        pathFindToPose(Constants.Game.Auto.Waypoints.kNote4Poses.noteScorePose()),
        launch()
      )
    )
    .withName("BackupShootPickup24");
  } 

  public Command backupScorePickup25() {
    PathPlannerPath pathToPickup5 = PathPlannerPath.fromPathFile("Pickup5");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload2")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch(), 
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup5, Constants.Drive.kPathFindingConstraints),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        pathFindToPose(Constants.Game.Auto.Waypoints.kNote5Poses.noteScorePose()),
        launch()
      )
    )
    .withName("BackupShootPickup25");
  } 

  public Command backupScorePickup26() {
    PathPlannerPath pathToPickup6 = PathPlannerPath.fromPathFile("Pickup6");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload2")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch(),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup6, Constants.Drive.kPathFindingConstraints),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        pathFindToPose(Constants.Game.Auto.Waypoints.kNote6Poses.noteScorePose()),
        launch()
      )
    )
    .withName("BackupShootPickup25");
  } 

  public Command backupScorePickup38() {
    PathPlannerPath pathToPickup8 = PathPlannerPath.fromPathFile("Pickup8");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToPose(Constants.Game.Auto.kNoteScoringPoses.get("Preload3")),
        launch(),
        Commands.parallel(
          pathFindToPose(Constants.Game.Auto.Waypoints.kNote3Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        launch(),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(pathToPickup8, Constants.Drive.kPathFindingConstraints),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        pathFindToPose(Constants.Game.Auto.Waypoints.kNote8Poses.noteScorePose()),
        launch()
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
      launch()
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
      ),
      launch()
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
      ),
      launch()
    )
    .withName("ScorePickup3");
  } 

  public Command scoreMoveout3() {
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      pathFindToPose(new Pose2d(2.75, 2.75, Rotation2d.fromDegrees(0)))
    )
    .withName("ScoreMoveout3");
  } 

  public Command scoreSubwooferAuto() {
    return m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionSubwoofer)
    .withTimeout(1.0)
    .andThen(
      m_gameCommmands.runLauncherAutoCommand()
    )
    .withName("ScoreSubwooferAuto");
  }
}
