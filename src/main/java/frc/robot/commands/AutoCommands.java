package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Enums.AutoPath;
import frc.robot.lib.common.Enums.AutoPose;
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
  // TODO: add timeouts to note pickup/scoring sequences that allows to move on to next pickup/score option if the note pickup is missed

  private PathPlannerPath path(AutoPath autoPath) {
    return Constants.Game.Auto.kPaths.get(autoPath);
  }

  private Pose2d pose(AutoPose autoPose) {
    return Constants.Game.Auto.kPoses.get(autoPose);
  }

  private Command pathFindToPose(Pose2d pose) {
    return 
    Commands.either(
      AutoBuilder.pathfindToPose(pose, Constants.Drive.kPathFindingConstraints),
      AutoBuilder.pathfindToPoseFlipped(pose, Constants.Drive.kPathFindingConstraints),
      () -> Robot.getAlliance() == Alliance.Blue
    )
    .withName("PathFindToPose");
  }

  private Command move(PathPlannerPath path) {
    return
    AutoBuilder.pathfindThenFollowPath(path, Constants.Drive.kPathFindingConstraints)
    .withName("MoveWithPath");
  }

  private Command move(Pose2d pose) {
    return
    pathFindToPose(pose)
    .withName("MoveToPose");
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
    .withName("PickupAtPose");
  }

  private Command score() {
    return
    m_gameCommmands.alignRobotToTargetCommand()
    .alongWith(m_gameCommmands.alignLauncherToTargetAutoCommand())
    .withTimeout(2.0)
    .andThen(m_gameCommmands.runLauncherAutoCommand())
    .withName("ScoreDynamicPosition");
  }

  public Command score(double launcherArmPosition) {
    return 
    m_gameCommmands.alignLauncherToPositionAutoCommand(launcherArmPosition)
    .withTimeout(2.0)
    .andThen(m_gameCommmands.runLauncherAutoCommand())
    .withName("ScoreFixedPosition");
  }

  private Command run() {
    return
    m_launcherRollerSubsystem.runCommand(() -> Constants.Launcher.kDefaultLauncherSpeeds)
    .withName("RunLauncherRollers");
  }

  // ============================================================

  public Command auto_0() {
    return 
    score(Constants.Launcher.kArmPositionSubwoofer)
    .deadlineWith(run())
    .withName("Auto_0");
  } 

  public Command auto_10_1() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(pose(AutoPose.Pickup1)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_10_1");
  }

  public Command auto_1_0_1() {
    return 
    Commands.sequence(
      move(pose(AutoPose.ScorePreload1)),
      score(),
      pickup(pose(AutoPose.Pickup1)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_1_0_1");
  } 

  public Command auto_1_0_1_4() {
    return 
    Commands.sequence(
      move(pose(AutoPose.ScorePreload1)),
      score(),
      pickup(pose(AutoPose.Pickup1)),
      score(), 
      pickup(path(AutoPath.Pickup4)),
      move(path(AutoPath.ScoreStage1)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_1_0_1_4");
  } 

  public Command auto_20_2() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(pose(AutoPose.Pickup2)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_20_2");
  }

  public Command auto_2_0_2() {
    return 
    Commands.sequence(
      move(pose(AutoPose.ScorePreload2)),
      score(),
      pickup(pose(AutoPose.Pickup2)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_2_0_2");
  } 

  public Command auto_30_3() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(pose(AutoPose.Pickup3)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_20_2");
  }

  public Command auto_3_0_3() {
    return 
    Commands.sequence(
      move(pose(AutoPose.ScorePreload3)),
      score(),
      pickup(pose(AutoPose.Pickup3)),
      score()
    )
    .deadlineWith(run())
    .withName("Auto_3_0_3");
  } 
}
