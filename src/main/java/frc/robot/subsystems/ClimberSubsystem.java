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
import frc.robot.lib.common.Utils;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotorLeft;
  private final CANSparkMax m_armMotorRight;

  private final RelativeEncoder m_armLeftEncoder;
  private final RelativeEncoder m_armRightEncoder;

  private final SparkPIDController m_armLeftPIDController;
  private final SparkPIDController m_armRightPIDController;

  private boolean m_hasInitialReset = false;
  
  public ClimberSubsystem() {
    m_armMotorLeft = new CANSparkMax(Constants.Climber.kArmMotorCANId, MotorType.kBrushless);
    m_armMotorLeft.restoreFactoryDefaults();
    m_armMotorLeft.setIdleMode(Constants.Climber.kArmMotorIdleMode); 
    m_armMotorLeft.setSmartCurrentLimit(Constants.Climber.kArmMotorCurrentLimit);
    m_armMotorLeft.setSecondaryCurrentLimit(Constants.Climber.kArmMotorCurrentLimit);
    m_armMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotorLeft.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.kArmMotorForwardSoftLimit); 
    m_armMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.kArmMotorReverseSoftLimit);

    m_armMotorLeft.burnFlash();

    m_armLeftEncoder = m_armMotorLeft.getEncoder();

    m_armLeftPIDController = m_armMotorLeft.getPIDController();
    m_armLeftPIDController.setP(Constants.Climber.kArmMotorPIDConstants.P());
    m_armLeftPIDController.setD(Constants.Climber.kArmMotorPIDConstants.D());
    m_armLeftPIDController.setOutputRange(Constants.Climber.kArmMotorMaxReverseOutput, Constants.Climber.kArmMotorMaxForwardOutput);

    m_armMotorRight = new CANSparkMax(Constants.Climber.kRollerMotorCANId, MotorType.kBrushless);
    m_armMotorRight.restoreFactoryDefaults();
    m_armMotorRight.setIdleMode(Constants.Climber.kArmMotorIdleMode); 
    m_armMotorRight.setSmartCurrentLimit(Constants.Climber.kArmMotorCurrentLimit);
    m_armMotorRight.setSecondaryCurrentLimit(Constants.Climber.kArmMotorCurrentLimit);
    m_armMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotorRight.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.kArmMotorForwardSoftLimit); 
    m_armMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotorRight.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.kArmMotorReverseSoftLimit);
    m_armMotorRight.setInverted(true);

    m_armMotorRight.burnFlash();

    m_armRightEncoder = m_armMotorRight.getEncoder();

    m_armRightPIDController = m_armMotorLeft.getPIDController();
    m_armRightPIDController.setP(Constants.Climber.kArmMotorPIDConstants.P());
    m_armRightPIDController.setD(Constants.Climber.kArmMotorPIDConstants.D());
    m_armRightPIDController.setOutputRange(Constants.Climber.kArmMotorMaxReverseOutput, Constants.Climber.kArmMotorMaxForwardOutput);

  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command moveArmUpCommand() {
    return 
    run(() -> {
      m_armLeftPIDController.setReference(Constants.Climber.kArmMotorForwardSoftLimit, ControlType.kPosition);
      m_armRightPIDController.setReference(Constants.Climber.kArmMotorForwardSoftLimit, ControlType.kPosition);
    })
    .withName("MoveClimberOut");
  }

  public Command moveArmDownCommand() {
    return 
    run(() -> {
      m_armLeftPIDController.setReference(Constants.Climber.kArmMotorReverseSoftLimit, ControlType.kPosition);
      m_armRightPIDController.setReference(Constants.Climber.kArmMotorReverseSoftLimit, ControlType.kPosition);
    })
    .withName("MoveClimberIn");
  }

  public Command resetCommand() {
    return
    startEnd(() -> {
      Utils.enableSoftLimits(m_armMotorLeft, false);
      Utils.enableSoftLimits(m_armMotorRight, false);
      m_armMotorLeft.set(-0.1);
      m_armMotorRight.set(-0.1);
    }, () -> {
      m_armLeftEncoder.setPosition(0);
      m_armRightEncoder.setPosition(0);
      m_armMotorLeft.set(0.0);
      m_armMotorRight.set(0.0);
      Utils.enableSoftLimits(m_armMotorLeft, true);
      Utils.enableSoftLimits(m_armMotorRight, true);
      m_hasInitialReset = true;
    })
    .withName("ResetClimb");
  }

  public boolean hasInitialReset() {
    return m_hasInitialReset;
  }

  public void reset() {
    m_armMotorLeft.set(0.0);
    m_armMotorRight.set(0.0);
  }

  private void updateTelemetry() {
    double armLeftPosition = m_armLeftEncoder.getPosition();
    SmartDashboard.putNumber("Robot/Climber/ArmLeft/Position", armLeftPosition);

    double armRightPosition = m_armRightEncoder.getPosition();
    SmartDashboard.putNumber("Robot/Climber/ArmRight/Position", armRightPosition);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
