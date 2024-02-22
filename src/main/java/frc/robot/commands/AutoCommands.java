package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.lib.sensors.ObjectSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
  private final GameCommands m_gameCommmands;
  private final GyroSensor m_gyroSensor;   
  private final ObjectSensor m_objectSensor; 
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;

  public AutoCommands(
    GameCommands gameCommands,
    GyroSensor gyroSensor, 
    ObjectSensor objectSensor, 
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem
  ) {
    m_gameCommmands = gameCommands;
    m_gyroSensor = gyroSensor;
    m_objectSensor = objectSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
  }

  private Command resetGyroCommand() {
    return Commands
    .runOnce(() -> m_gyroSensor.reset(m_poseSubsystem.getPose().getRotation().getDegrees()))
    .withName("ResetGyro"); 
  }

  public Command test3NoteAuto() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test1");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Test2");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("Test3");
    PathConstraints constraints = new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    return Commands
    .sequence(
      resetGyroCommand(), // ADD TO START OF ALL AUTOS
      //m_poseSubsystem.resetPoseCommand(), //path.getPreviewStartingHolonomicPose()
      AutoBuilder.followPath(path1),
      //AutoBuilder.pathfindThenFollowPath(path1, constraints),
      Commands.waitSeconds(1),
      AutoBuilder.followPath(path2),
      //AutoBuilder.pathfindThenFollowPath(path2, constraints),
      Commands.waitSeconds(2),
      AutoBuilder.pathfindThenFollowPath(path3, constraints),
      Commands.waitSeconds(2)
      //Commands.runOnce(() -> m_driveSubsystem.setLockState(LockState.LOCKED))
    )
    .withName("RunAutoTest3NoteAuto");
  }

  public Command testPath4() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test4");
    return Commands
    .sequence(
      resetGyroCommand(),
      AutoBuilder.followPath(path1)
    )
    .withName("");
  }

  public Command testPath5() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test5");
    return Commands
    .sequence(
      AutoBuilder.followPath(path1)
    )
    .withName("");
  }

  public Command testPath6() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test6");
    return Commands
    .sequence(
      AutoBuilder.followPath(path1)
    )
    .withName("testPath4");
  }
}
