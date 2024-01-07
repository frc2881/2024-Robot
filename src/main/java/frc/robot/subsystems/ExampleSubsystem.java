package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  // TODO: define subsystem elements and states

  public ExampleSubsystem() {
    // TODO: initialize the subsystem
  }

  @Override
  public void periodic() {
    // TODO: update the subsystem
    updateTelemetry();
  }

  public Command exampleCommand() {
    return Commands.runOnce(
      () -> {
        // TODO: call a private method or execute inline logic for subsystem
      })
      .withName("ExampleCommand");
  }

  public void reset() {
    // TODO: reset the subsystem
  }

  private void updateTelemetry() {
    // TODO: send subsystem telemetry data to the dashboard as needed
    // ex: SmartDashboard.putString("Robot/Example/String", "TEST");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: send subsystem data to be logged on the robot as needed
    // ex: builder.addDoubleProperty("Double", this::getSomeDoubleValue, null);
  }
}
