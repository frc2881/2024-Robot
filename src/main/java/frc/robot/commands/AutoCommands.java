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
    // TODO: configure timeout length for real auto progression and simulated auto runs for testing paths (set timeout to account for longest possible path to pickup)
    .withTimeout(4.0)
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
      pickup(path(AutoPath.Pickup1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_10_1");
  }

  public Command auto_1_0_1() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload1)),
      score(),
      pickup(path(AutoPath.Pickup1)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_1_0_1");
  } 

  public Command auto_1_0_1_4() {
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
    .withName("Auto_1_0_1_4");
  } 

  public Command auto_1_0_1_4_5() {
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
    .withName("Auto_1_0_1_4_5");
  } 

  public Command auto_1_0_1_5() {
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
    .withName("Auto_1_0_1_5");
  } 

  public Command auto_1_0_1_5_4() {
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
    .withName("Auto_1_0_1_5_6");
  } 

  public Command auto_1_0_1_5_6() {
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
    .withName("Auto_1_0_1_5_6");
  } 

  public Command auto_20_2() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup2)),
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
      pickup(path(AutoPath.Pickup2)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_2_0_2");
  } 

  public Command auto_2_0_2_6() {
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
    .withName("Auto_2_0_2_6");
  } 

  public Command auto_2_0_2_6_5() {
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
    .withName("Auto_2_0_2_6_7");
  } 

  public Command auto_2_0_2_6_7() {
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
    .withName("Auto_2_0_2_6_7");
  } 

  public Command auto_2_0_2_7() {
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
    .withName("Auto_2_0_2_7");
  } 

  public Command auto_2_0_2_7_6() {
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
    .withName("Auto_2_0_2_6_7");
  } 

  public Command auto_30_3() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_30_3");
  }

  public Command auto_3_0_3() {
    return 
    Commands.sequence(
      move(path(AutoPath.ScorePreload3)),
      score(),
      pickup(path(AutoPath.Pickup3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_3_0_3");
  } 

  public Command auto_3_0_3_82() {
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
    .withName("Auto_3_0_3_82");
  } 

  public Command auto_3_0_3_83() {
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
    .withName("Auto_3_0_3_83");
  } 

  public Command auto_3_0_3_82_62() {
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
    .withName("Auto_3_0_3_8_7");
  }

  // public Command auto_3_0_3_82_63() {
  //   return 
  //   Commands.sequence(
  //     move(path(AutoPath.ScorePreload3)),
  //     score(),
  //     pickup(path(AutoPath.Pickup3)),
  //     score(),
  //     pickup(path(AutoPath.Pickup8)),
  //     move(path(AutoPath.ScoreStage2)),
  //     score(),
  //     pickup(path(AutoPath.Pickup6)),
  //     move(path(AutoPath.ScoreStage3)),
  //     score()
  //   )
  //   .deadlineWith(start())
  //   .withName("Auto_3_0_3_8_7");
  // }

  public Command auto_3_0_3_82_72() {
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
    .withName("Auto_3_0_3_8_7");
  }

  public Command auto_3_0_3_82_73() {
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
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_3_0_3_8_7");
  }

  public Command auto_3_0_3_83_62() {
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
    .withName("Auto_3_0_3_8_7");
  }

  // public Command auto_3_0_3_83_63() {
  //   return 
  //   Commands.sequence(
  //     move(path(AutoPath.ScorePreload3)),
  //     score(),
  //     pickup(path(AutoPath.Pickup3)),
  //     score(),
  //     pickup(path(AutoPath.Pickup8)),
  //     move(path(AutoPath.ScoreStage3)),
  //     score(),
  //     pickup(path(AutoPath.Pickup6)),
  //     move(path(AutoPath.ScoreStage3)),
  //     score()
  //   )
  //   .deadlineWith(start())
  //   .withName("Auto_3_0_3_8_7");
  // }

  public Command auto_3_0_3_83_72() {
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
    .withName("Auto_3_0_3_8_7");
  }

  public Command auto_3_0_3_83_73() {
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
    .withName("Auto_3_0_3_8_7");
  }
  
  public Command auto_30_7() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup73)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_30_7");
  } 

  public Command auto_30_7_8() {
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
    .withName("Auto_30_8_7");
  } 

  public Command auto_30_8() {
    return 
    Commands.sequence(
      score(Constants.Launcher.kArmPositionSubwoofer),
      pickup(path(AutoPath.Pickup8)),
      move(path(AutoPath.ScoreStage3)),
      score()
    )
    .deadlineWith(start())
    .withName("Auto_30_8");
  }

  public Command auto_30_8_7() {
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
    .withName("Auto_30_8_7");
  } 
}
