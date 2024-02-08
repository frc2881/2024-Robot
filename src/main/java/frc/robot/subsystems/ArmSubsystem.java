package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public static enum RollerDirection { Inward, Outward; }

  private final CANSparkMax m_armMotor;
  private final SparkPIDController m_armPIDController;
  private final RelativeEncoder m_armEncoder;
  private final CANSparkMax m_rollerMotor;

  private boolean m_isAtPosition = false;
  
  public ArmSubsystem() {
    m_armMotor = new CANSparkMax(Constants.Arm.kArmMotorCANId, MotorType.kBrushless);
    // m_armMotor.restoreFactoryDefaults();
    // m_armMotor.setIdleMode(Constants.Arm.kArmMotorIdleMode); 
    // m_armMotor.setSmartCurrentLimit(Constants.Arm.kArmMotorCurrentLimit);
    // m_armMotor.setSecondaryCurrentLimit(Constants.Arm.kArmMotorCurrentLimit);
    // m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Arm.kArmMotorForwardSoftLimit); 
    // m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Arm.kArmMotorReverseSoftLimit);

    m_armEncoder = m_armMotor.getEncoder();
    // m_armEncoder.setPositionConversionFactor(Constants.Arm.kArmMotorPositionConversionFactor);
    // m_armEncoder.setVelocityConversionFactor(Constants.Arm.kArmMotorVelocityConversionFactor);
    
    m_armPIDController = m_armMotor.getPIDController();
    // m_armPIDController.setFeedbackDevice(m_armEncoder);
    // m_armPIDController.setP(Constants.Arm.kArmMotorPIDConstants.P);
    // m_armPIDController.setD(Constants.Arm.kArmMotorPIDConstants.D);
    // m_armPIDController.setOutputRange(Constants.Arm.kArmMotorMinOutput, Constants.Arm.kArmMotorMaxOutput);
    // m_armPIDController.setSmartMotionMaxVelocity(Constants.Arm.kArmMotorSmartMotionMaxVelocity, 0);
    // m_armPIDController.setSmartMotionMaxAccel(Constants.Arm.kArmMotorSmartMotionMaxAccel, 0);

    // TODO: get correct motor configuration for brushed/bag motor
    m_rollerMotor = new CANSparkMax(Constants.Arm.kRollerMotorCANId, MotorType.kBrushless);
    // m_rollerMotor.restoreFactoryDefaults();
    // m_rollerMotor.setIdleMode(Constants.Arm.kRollerMotorIdleMode); 
    // m_rollerMotor.setSmartCurrentLimit(Constants.Arm.kRollerMotorCurrentLimit);
    // m_rollerMotor.setSecondaryCurrentLimit(Constants.Arm.kRollerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command moveToPositionCommand(double position) {
    return
      runOnce(() -> {
        m_isAtPosition = false;
      })
      .andThen(
        this
          .run(() -> {
            m_armPIDController.setReference(position, ControlType.kSmartMotion);
            m_isAtPosition = Math.abs(m_armEncoder.getPosition() - position) < 0.1; // TODO: determine if this is correct tolerance
          })
          .until(() -> m_isAtPosition)
          .finallyDo(() -> m_armMotor.set(0.0))
      )
      .withName("MoveArmToPosition");
  }

  public Command runRollersCommand(RollerDirection direction) {
    return 
      run(() -> {
        // TODO: determine correct direction of travel for inward/outward with motor
        m_rollerMotor.set(
          direction == RollerDirection.Inward ? 
          Constants.Arm.kRollerMotorMaxOutput : 
          Constants.Arm.kRollerMotorMinOutput
        );
        // TODO: determine current limit to stop rollers when performing inward intake
      })
      .withName("RunArmRollers");
  }

  public Command resetCommand() {
    return 
      startEnd(
        () -> {
          m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
          m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
          m_armMotor.set(-0.1);
        }, 
        () -> {
          m_armEncoder.setPosition(0);
          m_armMotor.set(0.0);
          m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
          m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }
      )
      .withName("ResetArm");
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: send subsystem data to be logged on the robot as needed
    // ex: builder.addDoubleProperty("Double", this::getSomeDoubleValue, null);
  }
}
