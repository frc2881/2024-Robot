package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.lib.common.Enums.AutoPath;

public class AutoCommands {
  private final GameCommands m_gameCommmands; 

  public AutoCommands(GameCommands gameCommands) {
    m_gameCommmands = gameCommands;
  }

  private PathPlannerPath path(AutoPath autoPath) {
    return Constants.Game.Auto.kPaths.get(autoPath);
  }

  private Command move(PathPlannerPath path) {
    return
    AutoBuilder.pathfindThenFollowPath(path, Constants.Drive.kPathFindingConstraints)
    .withName("MoveWithPath");
  }
  
  private Command pickup(PathPlannerPath path) {
    return
    m_gameCommmands.runIntakeAutoCommand()
    .deadlineWith(move(path))
    .withTimeout(3.75)
    .withName("PickupWithPath");
  }

  private Command score() {
    return
    m_gameCommmands.alignRobotToTargetAutoCommand()
    .alongWith(m_gameCommmands.alignLauncherToSpeakerAutoCommand())
    .andThen(m_gameCommmands.runLauncherAutoCommand())
    .withName("ScoreDynamicPosition");
  }

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

  public Command auto0() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer)
    )
    .deadlineWith(start())
    .withName("Auto0");
  } 

  public Command auto10_1() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto10_1");
  }

  public Command auto1_0_1() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto1_0_1");
  } 

  public Command auto1_0_1_41() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score(), 
      pickup(path(AutoPath.Pickup4)),
      move(path(AutoPath.ScoreStage1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto1_0_1_41");
  } 

  public Command auto1_0_1_41_51() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score(), 
      pickup(path(AutoPath.Pickup4)),
      move(path(AutoPath.ScoreStage1)),
      score(),
      pickup(path(AutoPath.Pickup5)),
      move(path(AutoPath.ScoreStage1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto1_0_1_41_51");
  } 

  public Command auto1_0_1_51() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score(), 
      pickup(path(AutoPath.Pickup5)),
      move(path(AutoPath.ScoreStage1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto1_0_1_51");
  } 

  public Command auto1_0_1_51_41() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score(), 
      pickup(path(AutoPath.Pickup5)),
      move(path(AutoPath.ScoreStage1)),
      score(),
      pickup(path(AutoPath.Pickup4)),
      move(path(AutoPath.ScoreStage1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto1_0_1_51_41");
  } 

  public Command auto1_0_1_51_62() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score(), 
      pickup(path(AutoPath.Pickup5)),
      move(path(AutoPath.ScoreStage1)),
      score(),
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto1_0_1_51_62");
  } 

  public Command auto20_2() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto20_2");
  }

  public Command auto2_0_2() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(path(AutoPath.Pickup2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto2_0_2");
  } 

  public Command auto2_0_2_62() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(path(AutoPath.Pickup2)),
      score(), 
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto2_0_2_62");
  } 

  public Command auto2_0_2_62_51() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(path(AutoPath.Pickup2)),
      score(), 
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score(),
      pickup(path(AutoPath.Pickup5)),
      move(path(AutoPath.ScoreStage1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto2_0_2_62_51");
  } 

  public Command auto2_0_2_62_72() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(path(AutoPath.Pickup2)),
      score(), 
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score(),
      pickup(path(AutoPath.Pickup72)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto2_0_2_62_72");
  } 

  public Command auto2_0_2_72() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(path(AutoPath.Pickup2)),
      score(), 
      pickup(path(AutoPath.Pickup72)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto2_0_2_72");
  } 

  public Command auto2_0_2_72_62() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload2)),
      score(),
      pickup(path(AutoPath.Pickup2)),
      score(), 
      pickup(path(AutoPath.Pickup72)),
      move(path(AutoPath.ScoreStage2)),
      score(),
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto2_0_2_72_62");
  } 

  public Command auto30_3() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto30_3");
  }

  public Command auto30_73() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup73)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto30_73");
  } 

  public Command auto30_83() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto30_83");
  }

    public Command auto30_73_83() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup73)),
      move(path(AutoPath.ScoreStage3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto30_73_83");
  } 

  public Command auto30_83_73() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score(),
      pickup(path(AutoPath.Pickup73)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto30_83_73");
  } 

  public Command auto3_0_3() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3");
  } 

  // TOOD: create auto3_0_3_72
  // TODO: create auto3_0_3_73
  // TODO: create auto3_0_3_72_62
  // TODO: create auto3_0_3_73_83

  public Command auto3_0_3_82() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_82");
  } 

  public Command auto3_0_3_83() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_83");
  } 

  public Command auto3_0_3_82_62() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage2)),
      score(),
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_82_62");
  }

  public Command auto3_0_3_82_72() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage2)),
      score(),
      pickup(path(AutoPath.Pickup72)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_82_72");
  }

  public Command auto3_0_3_83_62() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score(),
      pickup(path(AutoPath.Pickup6)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_83_62");
  }

  public Command auto3_0_3_83_72() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score(),
      pickup(path(AutoPath.Pickup73)),
      move(path(AutoPath.ScoreStage2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_83_72");
  }

  public Command auto3_0_3_83_73() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score(),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score(),
      pickup(path(AutoPath.Pickup73)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto3_0_3_83_73");
  }
}
