package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;

public class LauncherRollerSubsystem extends SubsystemBase {
  private final CANSparkFlex m_topRollerMotor;
  private final CANSparkFlex m_bottomRollerMotor;

  public LauncherRollerSubsystem() {
    m_topRollerMotor = new CANSparkFlex(Constants.Launcher.kTopRollerMotorCANId, MotorType.kBrushless);
    m_topRollerMotor.setCANMaxRetries(10);
    Utils.validateREVLib(m_topRollerMotor.restoreFactoryDefaults());
    Utils.validateREVLib(m_topRollerMotor.setIdleMode(Constants.Launcher.kTopRollerMotorIdleMode)); 
    Utils.validateREVLib(m_topRollerMotor.setSmartCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit));
    Utils.validateREVLib(m_topRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit));
    Utils.validateREVLib(m_topRollerMotor.burnFlash());

    m_bottomRollerMotor = new CANSparkFlex(Constants.Launcher.kBottomRollerMotorCANId, MotorType.kBrushless);
    m_bottomRollerMotor.setCANMaxRetries(10);
    Utils.validateREVLib(m_bottomRollerMotor.restoreFactoryDefaults());
    Utils.validateREVLib(m_bottomRollerMotor.setIdleMode(Constants.Launcher.kBottomRollerMotorIdleMode)); 
    Utils.validateREVLib(m_bottomRollerMotor.setSmartCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit));
    Utils.validateREVLib(m_bottomRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit));
    Utils.validateREVLib(m_bottomRollerMotor.burnFlash());
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runCommand(Supplier<LauncherRollerSpeeds> rollerSpeeds) {
    return 
    startEnd(() -> {
      m_topRollerMotor.set(rollerSpeeds.get().top() * Constants.Launcher.kTopRollerMotorMaxReverseOutput);
      m_bottomRollerMotor.set(rollerSpeeds.get().bottom() * Constants.Launcher.kBottomRollerMotorMaxForwardOutput);
    },
    () -> { 
      m_topRollerMotor.set(0.0);
      m_bottomRollerMotor.set(0.0);
    })
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
