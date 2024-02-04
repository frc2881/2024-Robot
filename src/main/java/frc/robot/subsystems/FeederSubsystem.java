package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private final CANSparkMax m_rollerMotor;
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;

  public FeederSubsystem() {
    m_rollerMotor = new CANSparkMax(Constants.Feeder.kRollerCanId, MotorType.kBrushless);

    m_armMotor = new CANSparkMax(Constants.Feeder.kArmCanId, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake); 
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)Constants.Feeder.kForwardLimit); 
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)Constants.Feeder.kReverseLimit);
    m_armMotor.setSmartCurrentLimit(60);
    m_armMotor.setSecondaryCurrentLimit(60, 0);

    m_armEncoder = m_armMotor.getEncoder();
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runRollersCommand(Double speed) {
    return Commands.run(
      () -> {
        m_rollerMotor.set(speed);
      },
      this);
  }
  public Command moveArmCommand(Double speed) {
    return Commands.run(
      () -> {
        m_armMotor.set(speed);
      },
      this);
  }

  public Double getEncoderPosition(){
    return m_armEncoder.getPosition();
  }

  public void resetEncoder() {
    m_armEncoder.setPosition(0);
  }

  public void enableSoftLimitsCommand(boolean enable){
    if (enable) {
      m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    } else {
      m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }
  }

  public Command moveArmOverrideCommand() {
    return Commands.runOnce(
      () -> {
        enableSoftLimitsCommand(false);
      }
    )
    .andThen(
      moveArmCommand(-0.15)
    )
    .finallyDo(
      () -> {
        enableSoftLimitsCommand(true);
        resetEncoder();
        moveArmCommand(0.0);
      }
    )
    .withName("moveArmOverrideCommand");
  }

  public Command moveArmOut(Double speed) {
    return moveArmCommand(speed)
      .until(() -> getEncoderPosition() >= Constants.Feeder.kForwardLimit)
      .andThen(moveArmCommand(0.0));
  }

  public Command moveArmIn(Double speed) {
    return moveArmCommand(speed)
      .until(() -> getEncoderPosition() <= Constants.Feeder.kReverseLimit)
      .andThen(moveArmCommand(0.0));
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
