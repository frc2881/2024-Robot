package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.sensors.Gyro;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class AutoCommands {
    
    private DriveSubsystem m_driveSubsystem;
    private PoseSubsystem m_poseSubsystem;
    private Gyro m_gyro;

    public AutoCommands(DriveSubsystem driveSubsystem, PoseSubsystem poseSubsystem, Gyro gyro) {
      m_driveSubsystem = driveSubsystem;
      m_poseSubsystem = poseSubsystem;
      m_gyro = gyro;
    }

    public Command testPath() {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Test");
      PathConstraints constraints = new PathConstraints(
        1.0, 1.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
      return Commands.sequence(
        Commands.runOnce(() -> m_poseSubsystem.resetPose()), //path.getPreviewStartingHolonomicPose()
        //AutoBuilder.followPath(path)
        AutoBuilder.pathfindThenFollowPath(path, constraints)
        //Commands.runOnce(() -> m_driveSubsystem.setLockState(LockState.LOCKED))
      )
      .withName("RunAutoTestPath");
    }

    public Command testAuto() {
      return AutoBuilder.buildAuto("Test")
      .withName("RunAutoTestAuto");
    }
}
