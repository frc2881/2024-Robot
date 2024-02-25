package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherRollerSubsystem extends SubsystemBase {
  private final CANSparkFlex m_topRollerMotor;
  private final CANSparkFlex m_bottomRollerMotor;

  public record RollerSpeeds(double top, double bottom) {}

  public LauncherRollerSubsystem() {
    m_topRollerMotor = new CANSparkFlex(Constants.Launcher.kTopRollerMotorCANId, MotorType.kBrushless);
    m_topRollerMotor.restoreFactoryDefaults();
    m_topRollerMotor.setIdleMode(Constants.Launcher.kTopRollerMotorIdleMode); 
    m_topRollerMotor.setSmartCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);
    m_topRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);
    m_topRollerMotor.setInverted(true);

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

  public Command runCommand(Supplier<RollerSpeeds> rollerSpeeds) {
    return 
    startEnd(() -> {
      m_topRollerMotor.set(rollerSpeeds.get().top * Constants.Launcher.kTopRollerMotorMaxOutput);
      m_bottomRollerMotor.set(rollerSpeeds.get().bottom * Constants.Launcher.kBottomRollerMotorMaxOutput);
    },
    () -> { 
      m_topRollerMotor.set(0.0);
      m_bottomRollerMotor.set(0.0);
    })
    .withName("RunLauncherRollers");
  }

/*
 * Make sure you set to zero when it needs to finish as it does not set automatically
 */
  public Command runAutoCommand(Supplier<RollerSpeeds> rollerSpeeds) {
    return 
    startEnd(() -> {
      m_topRollerMotor.set(rollerSpeeds.get().top * Constants.Launcher.kTopRollerMotorMaxOutput);
      m_bottomRollerMotor.set(rollerSpeeds.get().bottom * Constants.Launcher.kBottomRollerMotorMaxOutput);
    },
    () -> {})
    .withName("RunLauncherRollers");
  }

  public void reset() {
    m_topRollerMotor.set(0.0);
    m_bottomRollerMotor.set(0.0);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Launcher/Roller/Top/Speed", m_topRollerMotor.get());
    SmartDashboard.putNumber("Robot/Launcher/Roller/Bottom/Speed", m_bottomRollerMotor.get());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
