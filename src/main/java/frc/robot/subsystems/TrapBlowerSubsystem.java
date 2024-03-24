package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.common.Utils;

public class TrapBlowerSubsystem extends SubsystemBase {
  private final CANSparkMax m_blowerMotor;

  public TrapBlowerSubsystem() {
    m_blowerMotor = new CANSparkMax(14, MotorType.kBrushless);
    //m_blowerMotor.setCANMaxRetries(10);
    Utils.validateREVLib(m_blowerMotor.restoreFactoryDefaults());
    Utils.validateREVLib(m_blowerMotor.setIdleMode(IdleMode.kCoast)); 
    Utils.validateREVLib(m_blowerMotor.setSmartCurrentLimit(40));
    Utils.validateREVLib(m_blowerMotor.setSecondaryCurrentLimit(40));
    Utils.validateREVLib(m_blowerMotor.burnFlash());
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runCommand() {
    return 
    startEnd(() -> {
      m_blowerMotor.set(1.0);
    },
    () -> { 
      m_blowerMotor.set(0.0);
    })
    .withName("RunLauncherRollers");
  }

  public void reset() {
    m_blowerMotor.set(0.0);
  }

  private void updateTelemetry() {
    //SmartDashboard.putNumber("Robot/Launcher/Roller/Top/Speed", m_blowerMotor.get());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
