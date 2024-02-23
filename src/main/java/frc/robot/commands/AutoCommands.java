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

  public Command testPath() {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test");
    return Commands
    .sequence(
      //Commands.runOnce(() -> m_poseSubsystem.resetPose(path1.getPreviewStartingHolonomicPose())),
      AutoBuilder.followPath(path1)
    )
    .withName("Test");
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
