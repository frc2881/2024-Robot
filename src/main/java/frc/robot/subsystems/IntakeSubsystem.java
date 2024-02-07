package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static enum BeltDirection { Forward, Backward; }
  public static enum RollerDirection { Inward, Outward; }

  private final CANSparkMax m_beltMotor;
  private final CANSparkMax m_rollerMotor;

  public IntakeSubsystem() {
    m_beltMotor = new CANSparkMax(Constants.Intake.kBeltMotorCANId, MotorType.kBrushless);
    m_beltMotor.restoreFactoryDefaults();
    m_beltMotor.setIdleMode(Constants.Intake.kBeltMotorIdleMode); 
    m_beltMotor.setSmartCurrentLimit(Constants.Intake.kBeltMotorCurrentLimit);
    m_beltMotor.setSecondaryCurrentLimit(Constants.Intake.kBeltMotorCurrentLimit);

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

  public Command runBelts(BeltDirection direction) {
    return 
      run(() -> {
        // TODO: determine correct direction of travel for forward/backward with motor
        m_beltMotor.set(
          direction == BeltDirection.Forward ? 
          Constants.Intake.kBeltMotorMaxOutput : 
          Constants.Intake.kBeltMotorMinOutput
        );
      })
      .withName("RunIntakeBelts");
  }

  public Command runRollers(RollerDirection direction) {
    return 
      run(() -> {
        // TODO: determine correct direction of travel for inward/outward with motor
        m_rollerMotor.set(
          direction == RollerDirection.Inward ? 
          Constants.Intake.kRollerMotorMaxOutput : 
          Constants.Intake.kRollerMotorMinOutput
        );
      })
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
