package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static enum BeltDirection { Forward, Backward; }
  public static enum RollerDirection { Inward, Outward; }

  private final CANSparkMax m_topBeltMotor;
  private final CANSparkMax m_bottomBeltMotor;
  private final CANSparkMax m_rollerMotor;

  public IntakeSubsystem() {
    m_topBeltMotor = new CANSparkMax(Constants.Intake.kTopBeltMotorCANId, MotorType.kBrushless);
    m_topBeltMotor.restoreFactoryDefaults();
    m_topBeltMotor.setIdleMode(Constants.Intake.kTopBeltMotorIdleMode); 
    m_topBeltMotor.setSmartCurrentLimit(Constants.Intake.kTopBeltMotorCurrentLimit);
    m_topBeltMotor.setSecondaryCurrentLimit(Constants.Intake.kTopBeltMotorCurrentLimit);

    m_bottomBeltMotor = new CANSparkMax(Constants.Intake.kBottomBeltMotorCANId, MotorType.kBrushless);
    m_bottomBeltMotor.restoreFactoryDefaults();
    m_bottomBeltMotor.setIdleMode(Constants.Intake.kBottomBeltMotorIdleMode); 
    m_bottomBeltMotor.setSmartCurrentLimit(Constants.Intake.kBottomBeltMotorCurrentLimit);
    m_bottomBeltMotor.setSecondaryCurrentLimit(Constants.Intake.kBottomBeltMotorCurrentLimit);

    m_rollerMotor = new CANSparkMax(Constants.Intake.kRollerMotorCANId, MotorType.kBrushless);
    m_rollerMotor.restoreFactoryDefaults();
    m_rollerMotor.setIdleMode(Constants.Intake.kRollerMotorIdleMode); 
    m_rollerMotor.setSmartCurrentLimit(Constants.Intake.kRollerMotorCurrentLimit);
    m_rollerMotor.setSecondaryCurrentLimit(Constants.Intake.kRollerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runTopBelts(BeltDirection direction) {
    return 
      Commands.run(() -> {
        // TODO: determine correct direction of travel for forward/backward with motor
        m_topBeltMotor.set(
          direction == BeltDirection.Forward ? 
          Constants.Intake.kTopBeltMotorMaxOutput : 
          Constants.Intake.kTopBeltMotorMinOutput
        );
      })
      .finallyDo(() -> m_topBeltMotor.set(0.0))
      .withName("RunIntakeTopBelts");
  }

  public Command runBottomBelts(BeltDirection direction) {
    return 
      Commands.run(() -> {
        // TODO: determine correct direction of travel for forward/backward with motor
        m_bottomBeltMotor.set(
          direction == BeltDirection.Forward ? 
          Constants.Intake.kBottomBeltMotorMaxOutput : 
          Constants.Intake.kBottomBeltMotorMinOutput
        );
      })
      .finallyDo(() -> m_bottomBeltMotor.set(0.0))
      .withName("RunIntakeBottomBelts");
  }

  public Command runRollers(RollerDirection direction) {
    return 
      Commands.run(() -> {
        // TODO: determine correct direction of travel for inward/outward with motor
        m_rollerMotor.set(
          direction == RollerDirection.Inward ? 
          Constants.Intake.kRollerMotorMaxOutput : 
          Constants.Intake.kRollerMotorMinOutput
        );
      })
      .finallyDo(() -> m_rollerMotor.set(0.0))
      .withName("RunIntakeRollers");
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
