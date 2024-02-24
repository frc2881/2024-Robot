package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherRollerSubsystem extends SubsystemBase {
  private final CANSparkFlex m_topRollerMotor;
  private final CANSparkFlex m_bottomRollerMotor;

  public LauncherRollerSubsystem() {
    m_topRollerMotor = new CANSparkFlex(Constants.Launcher.kTopRollerMotorCANId, MotorType.kBrushless);
    m_topRollerMotor.restoreFactoryDefaults();
    m_topRollerMotor.setIdleMode(Constants.Launcher.kTopRollerMotorIdleMode); 
    m_topRollerMotor.setSmartCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);
    m_topRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);

    m_bottomRollerMotor = new CANSparkFlex(Constants.Launcher.kBottomRollerMotorCANId, MotorType.kBrushless);
    m_bottomRollerMotor.restoreFactoryDefaults();
    m_bottomRollerMotor.setIdleMode(Constants.Launcher.kBottomRollerMotorIdleMode); 
    m_bottomRollerMotor.setSmartCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit);
    m_bottomRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  // TODO: fix this subsystem to run top motor inverted and use only max output value for both motors in order to prevent ripping notes apart due to incorrect values sent in
  public Command runRollersCommand(double[] rollerSpeeds) {
    return 
    startEnd(() -> {
      SmartDashboard.putNumber("Robot/Launcher/Roller/Speed", rollerSpeeds[0]);
      m_topRollerMotor.set(rollerSpeeds[0] * Constants.Launcher.kTopRollerMotorMaxOutput);
      m_bottomRollerMotor.set(rollerSpeeds[1] * Constants.Launcher.kBottomRollerMotorMaxOutput);
    },
    () -> { 
      m_topRollerMotor.set(0.0);
      m_bottomRollerMotor.set(0.0);
    })
    .withName("RunLauncherRollers");
  }

  private void updateTelemetry() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
