package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Records.AutoPoses;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
  private final GameCommands m_gameCommmands;
  private final GyroSensor m_gyroSensor;   
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor; 
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;
  private final ClimberSubsystem m_climberSubsystem;

  private int m_i;

  public AutoCommands(
    GameCommands gameCommands,
    GyroSensor gyroSensor, 
    BeamBreakSensor launcherBottomBeamBreakSensor,
    BeamBreakSensor launcherTopBeamBreakSensor,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherArmSubsystem launcherArmSubsystem,
    LauncherRollerSubsystem launcherRollerSubsystem,
    ClimberSubsystem climberSubsystem
  ) {
    m_gameCommmands = gameCommands; 
    m_gyroSensor = gyroSensor;
    m_launcherBottomBeamBreakSensor = launcherBottomBeamBreakSensor;
    m_launcherTopBeamBreakSensor = launcherTopBeamBreakSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherArmSubsystem = launcherArmSubsystem;
    m_launcherRollerSubsystem = launcherRollerSubsystem;
    m_climberSubsystem = climberSubsystem;
  }

  // private Command pickupAndScoreNote(AutoPoses autoPoses) {
  //   return Commands.sequence(
  //     Commands.parallel(
  //       pathFindToNotePose(autoPoses.notePickupPose())
  //       //m_gameCommmands.runIntakeAutoCommand()
  //     )
  //     .unless(() -> autoPoses.notePickupPose().getX() == 0.0),
  //     Commands.sequence(
  //       pathFindToNotePose(autoPoses.noteScorePose())
  //     )
  //     .unless(() -> autoPoses.noteScorePose().getX() == autoPoses.notePickupPose().getX()), 
  //     Commands.parallel(
  //       m_gameCommmands.alignRobotToTargetCommand().withTimeout(1.0),
  //       m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(1.0)
  //     ),
  //     // m_gameCommmands.runLauncherAutoCommand()
  //     // .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
  //     new WaitCommand(1.0)
  //   )
  //   .withName("PickupAndScoreNote");
  // }

  // private Command runAutos(AutoPoses[] autoPoses) {
  //   return Commands.repeatingSequence(
  //     pickupAndScoreNote(autoPoses[m_i]),
  //     Commands.runOnce(
  //       () -> {
  //         m_i = m_i + 1;
  //       }
  //     )
  //   )
  //   .until(() -> m_i >= autoPoses.length)
  //   .beforeStarting(
  //     () -> {
  //       m_i = 1;
  //     }
  //   );
  // }

  // public Command runAuto(boolean isScoreAtSubwoofer, AutoPoses[] notesPoses) {
  //   return Commands.parallel(
  //     //m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
  //     Commands.sequence(
  //       Commands.either(
  //         scoreSubwooferAuto(), 
  //         pickupAndScoreNote(notesPoses[0]), 
  //         () -> isScoreAtSubwoofer),
  //       runAutos(notesPoses)
  //     )
  //   );
  // }

  private Command pathFindToNotePose(Pose2d notePose) {
    return Commands.either(
      AutoBuilder.pathfindToPose(notePose, Constants.Drive.kPathFindingConstraints),
      AutoBuilder.pathfindToPoseFlipped(notePose, Constants.Drive.kPathFindingConstraints),
      () -> Robot.getAlliance() == Alliance.Blue
    )
    .withName("PathFindToNotePickup");
  }

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
          m_gameCommmands.alignRobotToTargetCommand( ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
    )
    .withName("BackupScorePickup1");
  } 

  // public Command dynamicBackupScorePickup1() {
  //   return Commands.parallel(
  //     m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
  //     Commands.sequence(
  //       pickupAndScoreNote(Constants.Game.Field.AutoWaypoints.kNotePreload1Poses),
  //       pickupAndScoreNote(Constants.Game.Field.AutoWaypoints.kNote1Poses)
  //     )
  //   )
  //   .withName("DynamicBackupScorePickup1");
  // } 

  public Command backupScorePickup2() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload2Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote2Poses.notePickupPose()),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand( ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
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
          m_gameCommmands.alignRobotToTargetCommand( ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
    )
    .withName("BackupScorePickup3");
  } 

  public Command backupScorePickup14() {
    PathPlannerPath pathToPickup4 = PathPlannerPath.fromPathFile("Pickup4");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload1Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
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
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote4Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
        // new WaitCommand(2.0)
      )
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
    )
    .withName("BackupShootPickup14");
  } 

  public Command backupScorePickup15() {
    PathPlannerPath pathToPickup5 = PathPlannerPath.fromPathFile("Pickup5");
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNotePreload1Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
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
        pathFindToNotePose(Constants.Game.Field.AutoWaypoints.kNote5Poses.noteScorePose()),
        Commands.parallel(
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0),
          m_gameCommmands.alignRobotToTargetCommand()
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      )
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
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
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
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
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
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
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
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
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
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
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
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
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
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
          m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0)
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
      // TODO: have launcher rollers / belts stop after last auto scoring sequence is completed
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
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0),
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
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0),
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
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0),
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) 
    )
    .withName("ScorePickup3");
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
}
