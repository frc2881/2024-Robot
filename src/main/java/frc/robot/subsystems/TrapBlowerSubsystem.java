package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapBlowerSubsystem extends SubsystemBase {
  private final CANSparkMax m_blowerMotor;

  public TrapBlowerSubsystem() {
    m_blowerMotor = new CANSparkMax(14, MotorType.kBrushless);
    m_blowerMotor.restoreFactoryDefaults();
    m_blowerMotor.setIdleMode(IdleMode.kCoast); 
    m_blowerMotor.setSmartCurrentLimit(60);
    m_blowerMotor.setSecondaryCurrentLimit(60);
    m_blowerMotor.burnFlash();
  }

  // TODO: Make an auto align for trap using last years node code

  public Command runCommand() {
    return 
    startEnd(() -> {
      m_blowerMotor.set(1.0);
    },
    () -> { 
      m_blowerMotor.set(0.0);
    })
    .withName("RunTrapBlower");
  }

  public void reset() {
    m_blowerMotor.set(0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
