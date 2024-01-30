package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
    private GyroSensor m_gyroSensor;    
    private DriveSubsystem m_driveSubsystem;
    private PoseSubsystem m_poseSubsystem;

    public AutoCommands(GyroSensor gyroSensor, DriveSubsystem driveSubsystem, PoseSubsystem poseSubsystem) {
      m_gyroSensor = gyroSensor;
      m_driveSubsystem = driveSubsystem;
      m_poseSubsystem = poseSubsystem;
    }

    private Command resetGyroCommand() {
      return Commands.runOnce(
        () -> m_gyroSensor.reset(m_poseSubsystem.getPose().getRotation().getDegrees())
      ); 
    }

    public Command test3NoteAuto() {
      PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test1");
      PathPlannerPath path2 = PathPlannerPath.fromPathFile("Test2");
      PathPlannerPath path3 = PathPlannerPath.fromPathFile("Test3");
      PathConstraints constraints = new PathConstraints(
        2.0, 2.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
      return Commands.sequence(
        resetGyroCommand(), // ADD TO START OF ALL AUTOS
        //m_poseSubsystem.resetPoseCommand(), //path.getPreviewStartingHolonomicPose()
        AutoBuilder.followPath(path1),
        //AutoBuilder.pathfindThenFollowPath(path1, constraints),
        new WaitCommand(1).withName("Wait 1"),
        AutoBuilder.followPath(path2).withName("Path 2"),
        //AutoBuilder.pathfindThenFollowPath(path2, constraints),
        new WaitCommand(2),
        AutoBuilder.pathfindThenFollowPath(path3, constraints),
        new WaitCommand(2)
        //Commands.runOnce(() -> m_driveSubsystem.setLockState(LockState.LOCKED))
      )
      .withName("RunAutoTest3NoteAuto");
    }

    public Command testPath2() {
      PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test4");
      return Commands.sequence(
        resetGyroCommand(),
        AutoBuilder.followPath(path1)
      );
    }

    public Command testPath3() {
      PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test5");
      return Commands.sequence(
        AutoBuilder.followPath(path1)
      );
    }

    public Command testPath4() {
      PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test6");
      return Commands.sequence(
        AutoBuilder.followPath(path1)
      );
    }
}
