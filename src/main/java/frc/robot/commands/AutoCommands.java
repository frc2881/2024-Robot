package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Records.AutoPoses;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.common.Utils;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.DistanceSensor;
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
  private final LightsController m_lightsController;

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
    ClimberSubsystem climberSubsystem,
    LightsController lightsController
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
    m_lightsController = lightsController;
  }

  // TODO: update all autos to use path finding constraints from constants vs. inline

  // TODO: run gyro reset at the END of auto commands instead of at start
  public Command resetGyroCommand() { 
    return Commands
    .runOnce(() -> m_gyroSensor.reset(Utils.wrapAngle(m_poseSubsystem.getPose().getRotation().getDegrees())))
    .withName("ResetGyro"); 
  }

  private Command pathFindToNotePickup(Pose2d notePickupPose) {
    return Commands.either(
      AutoBuilder.pathfindToPose(notePickupPose, Constants.Drive.kPathFindingConstraints),
      AutoBuilder.pathfindToPoseFlipped(notePickupPose, Constants.Drive.kPathFindingConstraints),
      () -> Robot.getAlliance() == Alliance.Blue
    )
    .withName("PathFindToNotePickup");
  }

  private Command pickupAndScoreNote(AutoPoses autoPoses) {
    return Commands.sequence(
      Commands.parallel(
        pathFindToNotePickup(autoPoses.notePickupPose())
        //m_gameCommmands.runIntakeCommand()
      )
      .unless(() -> autoPoses.notePickupPose().getX() == 0.0),
      Commands.sequence(
        pathFindToNotePickup(autoPoses.noteScorePose())
      )
      .unless(() -> autoPoses.noteScorePose().getX() == autoPoses.notePickupPose().getX()), 
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand().withTimeout(1.0),
        m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(1.0) // TODO: add command get launcher pos
      ),
      // m_gameCommmands.runLauncherAutoCommand()
      // .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget())
      new WaitCommand(2.0)
    )
    .withName("PickupAndScoreNote");
  }

  // private Command runAutos(AutoPoses[] autoPoses) {
  //   return Commands.repeatingSequence(
  //     pickupAndScoreNote(autoPoses[i])
  //   ).beforeStarting(
  //     m_i = 1;
  //   )

  //   if (autoPoses.length > 1) {
  //     for (int i = 1; i < autoPoses.length; i++) {
        
  //     }
  //   }
  // } // TODO: Rework to be command

  // public Command runAuto(boolean isScoreAtSubwoofer, AutoPoses[] notesPoses) {
  //   return Commands.parallel(
  //     //m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
  //     Commands.sequence(
  //       Commands.either(
  //         scoreSubwooferAuto(), 
  //         pickupAndScoreNote(notesPoses[0]), 
  //         () -> isScoreAtSubwoofer),
  //       Commands.runOnce(
  //         () -> runAutos(notesPoses)
  //       )
  //     )
  //   ); // TODO: figure out whats wrong with adding reset gyro
  // }

  public Command scorePickup1() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup1");
    PathConstraints constraints = new PathConstraints(3, 3, 540.00, 720.00);
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
        m_gameCommmands.runIntakeCommand()
      ),
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0),
        m_gameCommmands.alignLauncherToPositionAutoCommand(Constants.Launcher.kArmPositionMidRange)
      ),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
    )
    .withName("ScorePickup1");
  } 

  public Command backupScorePickup1() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("BackupPickup1");
    PathConstraints constraints = new PathConstraints(3, 3, 540.00, 720.00);
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.8, 0.8)),
      Commands.sequence(
        Commands.parallel(
          Commands.either(
            AutoBuilder.pathfindToPose(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints),
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints),
            () -> Robot.getAlliance() == Alliance.Blue
          ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
          m_gameCommmands.runIntakeAutoCommand()
        ),
        // TODO: Make it so if it doesn't have a note in launcher, eject
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand( ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.runLauncherAutoCommand()
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
      )
    )
 
    .withName("BackupScorePickup1");
  } 

  public Command backupScorePickup14() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("BackupPickup1");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("BackupPickup4");
    PathConstraints constraints = new PathConstraints(3, 3, 540.00, 720.00);
    return Commands.parallel(
      m_launcherRollerSubsystem.runCommand(() -> new LauncherRollerSpeeds(0.7, 0.7)),
      Commands.sequence(
        Commands.parallel(
          Commands.either(
            AutoBuilder.pathfindToPose(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints),
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(45)), constraints), // Go to first position
            () -> Robot.getAlliance() == Alliance.Blue
          ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0) // move arm to blue line position
        ),
        m_gameCommmands.alignRobotToTargetCommand( ), // Align robot
        m_gameCommmands.runLauncherAutoCommand() // SHOOT FIRST NOTE
        .until(() -> !m_launcherTopBeamBreakSensor.hasTarget()),
        Commands.parallel(
          AutoBuilder.pathfindThenFollowPath(path1, constraints), // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        Commands.parallel(
          m_gameCommmands.alignRobotToTargetCommand( ), // align to speaker
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0) // align launcher to speaker
        ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT SECOND NOTE
        .until(() -> !m_launcherTopBeamBreakSensor.hasTarget()), // TODO: Create run launcher command that stops when note shot
        Commands.parallel(
          Commands.either(
            AutoBuilder.pathfindToPose(new Pose2d(8.6, 7.16, Rotation2d.fromDegrees(0)), constraints),
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(8.6, 7.16, Rotation2d.fromDegrees(0)), constraints),
            () -> Robot.getAlliance() == Alliance.Blue
          ), // grab next note
          m_gameCommmands.runIntakeAutoCommand() // grab next note
        ),
        Commands.parallel(
          Commands.either(
            AutoBuilder.pathfindToPose(new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)), constraints),
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)), constraints),
            () -> Robot.getAlliance() == Alliance.Blue
          ),
          m_gameCommmands.alignLauncherToTargetAutoCommand().withTimeout(2.0)
        ),
        m_gameCommmands.alignRobotToTargetCommand( ),
        m_gameCommmands.runLauncherAutoCommand() // SHOOT THIRD NOTE
        .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
      )
    )
    .withName("BackupShootPickup14");
  } 

  public Command scorePickup2() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup2");
    PathConstraints constraints = new PathConstraints(1.5, 1.5, 540.00, 720.00);
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
        m_gameCommmands.runIntakeCommand()
      )
      .withTimeout(5.0), // TODO: make it so it ends when intake is done (race???)
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0), // TODO: Make it so it ends when at position, not when timeouts over
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
    )
 
    .withName("ScorePickup2");
  } 

  public Command scorePickup3() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Pickup3");
    PathConstraints constraints = new PathConstraints(1.5, 1.5, 540.00, 720.00);
    return Commands
    .sequence(
      scoreSubwooferAuto(),
      Commands.parallel(
        AutoBuilder.pathfindThenFollowPath(path1, Constants.Drive.kPathFindingConstraints),
        m_gameCommmands.runIntakeCommand()
      )
      .withTimeout(5.0), // TODO: make it so it ends when intake is done (race???)
      // TODO: Make it so if it doesn't have a note in launcher, eject
      Commands.parallel(
        m_gameCommmands.alignRobotToTargetCommand( ).withTimeout(2.0), // TODO: Make it so it ends when at position, not when timeouts over
        m_gameCommmands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true)
      )
      .withTimeout(1.0),
      m_gameCommmands.runLauncherCommand()
      .until(() -> !m_launcherBottomBeamBreakSensor.hasTarget() && !m_launcherTopBeamBreakSensor.hasTarget()) // TODO: Create run launcher command that stops when note shot
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
