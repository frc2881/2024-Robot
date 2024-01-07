package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.LockState;

public class AutoCommands {
    
    private DriveSubsystem m_driveSubsystem;

    public AutoCommands(DriveSubsystem driveSubsystem) {
      this.m_driveSubsystem = driveSubsystem;
    }

    public Command testPath() {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Test");
      return Commands.sequence(
        Commands.runOnce(() -> m_driveSubsystem.resetPose(path.getPreviewStartingHolonomicPose())),
        AutoBuilder.followPathWithEvents(path),
        Commands.runOnce(() -> m_driveSubsystem.setLockState(LockState.LOCKED))
      )
      .withName("RunAutoTestPath");
    }

    public Command testAuto() {
      return AutoBuilder.buildAuto("Test")
      .withName("RunAutoTestAuto");
    }
}
