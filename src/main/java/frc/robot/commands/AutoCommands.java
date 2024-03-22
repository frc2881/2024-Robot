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
import frc.robot.lib.common.Enums.AutoPath;
import frc.robot.lib.common.Enums.AutoPose;

public class AutoCommands {
  private final GameCommands m_gameCommmands; 

  public AutoCommands(GameCommands gameCommands) {
    m_gameCommmands = gameCommands;
  }

  // TODO: migate all note pickup paths to use pathFindThenFollowPath methods and only use pathFindToPose for initial move out and preload scoring
  // TODO: add timeouts to note pickup/scoring sequences that allows to move on to next pickup/score option if the note pickup is missed

  private PathPlannerPath path(AutoPath autoPath) {
    return Constants.Game.Auto.kPaths.get(autoPath);
  }

  private Pose2d pose(AutoPose autoPose) {
    return Constants.Game.Auto.kPoses.get(autoPose);
  }

  private Command move(PathPlannerPath path) {
    return
    AutoBuilder.pathfindThenFollowPath(path, Constants.Drive.kPathFindingConstraints)
    .withName("MoveWithPath");
  }

  private Command move(Pose2d pose) {
    return
    Commands.either(
      AutoBuilder.pathfindToPose(pose, Constants.Drive.kPathFindingConstraints),
      AutoBuilder.pathfindToPoseFlipped(pose, Constants.Drive.kPathFindingConstraints),
      () -> Robot.getAlliance() == Alliance.Blue
    )
    .withName("MoveToPose");
  }

  private Command pickup(PathPlannerPath path) {
    return
    m_gameCommmands.runIntakeAutoCommand()
    .deadlineWith(move(path))
    // TODO: configure timeout length for real auto progression and simulated auto runs for testing paths
    .withTimeout(3.0)
    .withName("PickupWithPath");
  }

  private Command pickup(Pose2d pose) {
    return
    m_gameCommmands.runIntakeAutoCommand()
    .deadlineWith(move(pose))
    // TODO: configure timeout length for real auto progression and simulated auto runs for testing paths
    .withTimeout(3.0)
    .withName("PickupAtPose");
  }

  private Command score() {
    return
    m_gameCommmands.alignRobotToTargetAutoCommand()
    .alongWith(m_gameCommmands.alignLauncherToTargetAutoCommand())
    .andThen(m_gameCommmands.runLauncherAutoCommand())
    .withName("ScoreDynamicPosition");
  }

  // TODO: test that the predicate for subwoofer location only skips the robot alignment and not the launcher also
  public Command score(double launcherArmPosition) {
    return 
    m_gameCommmands.alignRobotToTargetAutoCommand()
    .onlyIf(() -> launcherArmPosition != Constants.Launcher.kArmPositionSubwoofer)
    .alongWith(m_gameCommmands.alignLauncherToPositionAutoCommand(launcherArmPosition))
    .andThen(m_gameCommmands.runLauncherAutoCommand())
    .withName("ScoreFixedPosition");
  }

  private Command start() {
    return
    m_gameCommmands.startLauncherRollersAutoCommand()
    .withName("StartLauncherRollersAuto");
  }

  /*
   * ######################################################################
   * ################################ AUTOS ###############################
   * ######################################################################
   */

   public Command testAuto() {
    return Commands.sequence(
      move(new Pose2d(4.0, 5.5, Rotation2d.fromDegrees(0)))
    );
   }

  public Command auto_0() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer)
    )
    .deadlineWith(start())
    .withName("Auto_0");
  } 

  public Command auto_10_1() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(pose(AutoPose.Pickup1)),
      score()
    )
    .deadlineWith(start())
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
    .deadlineWith(start())
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
    .deadlineWith(start())
    .withName("Auto_1_0_1_4");
  } 

  public Command auto_20_2() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(pose(AutoPose.Pickup2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_20_2");
  }

  public Command auto_2_0_2() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(pose(AutoPose.Pickup2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_2_0_2");
  } 

  public Command auto_30_3() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(pose(AutoPose.Pickup3)),
      score()
    )
    .deadlineWith(start())
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
    .deadlineWith(start())
    .withName("Auto_3_0_3");
  } 
}
