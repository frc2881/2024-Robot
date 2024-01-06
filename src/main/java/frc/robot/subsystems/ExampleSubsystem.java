package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.logging.Logger;

public class ExampleSubsystem extends SubsystemBase {

  public ExampleSubsystem() {}

  public Command exampleCommand() {
    return Commands.runOnce(
      () -> {
        Logger.log("Running ExampleCommand");
      })
      .withName("ExampleCommand");
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Example/Test", "TEST");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
