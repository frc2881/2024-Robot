package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotorLeft;
  private final CANSparkMax m_armMotorRight;

  private final Servo m_brakeServo;

  private final RelativeEncoder m_armLeftEncoder;
  private final RelativeEncoder m_armRightEncoder;

  private final SparkPIDController m_armLeftPIDController;

  private boolean m_hasInitialZeroReset = false;
  public boolean m_isBrakeApplied = false;
  public boolean m_isBeamBreakTriggered = false;
  
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
    m_armMotorLeft.setInverted(true);

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
    m_armMotorRight.follow(m_armMotorLeft, true);

    m_armMotorRight.burnFlash();

    m_armRightEncoder = m_armMotorRight.getEncoder();

    m_brakeServo = new Servo(9); // update
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command moveArmUpCommand() {
    return startEnd(
      () ->  m_armMotorLeft.set(0.6), 
      () ->  m_armMotorLeft.set(0.0)
    )
    .withName("moveArmTest");
  }

  public Command moveArmToTopCommand() {
    return moveArmUpCommand()
    .until(() -> m_armLeftEncoder.getPosition() >= Constants.Climber.kArmMotorForwardSoftLimit);
  }

  public Command moveArmDownCommand() {
    return startEnd(
      () ->  m_armMotorLeft.set(-0.5), 
      () ->  m_armMotorLeft.set(0.0)
    )
    .withName("moveArmTest");
  }

  public Command moveArmToDownCommand() {
    return moveArmDownCommand()
    .until(() -> m_armLeftEncoder.getPosition() <= Constants.Climber.kArmMotorMaxReverseOutput);
  }

  public Command unlockArmCommand() {
    return Commands.runOnce(
      () -> {
        m_brakeServo.setPosition(1.0);
        m_isBrakeApplied = false;
      }
    );
  }

  public Command lockArmCommand() {
    return Commands.runOnce(
      () -> {
        m_brakeServo.setPosition(0);
        m_isBrakeApplied = true;
      }
    );
  }

  public Command testArmCommand() {
    return Commands.sequence(
      unlockArmCommand(),
      resetZeroCommand()
    );
  }
  
  public void moveArmToStartingPosition() {
    m_armLeftPIDController.setReference(Constants.Climber.kArmPositionStarting, ControlType.kPosition);
  }

  public Command moveArmToStartingPositionCommand() {
    return Commands.runOnce(
      () -> m_armLeftPIDController.setReference(Constants.Climber.kArmPositionStarting, ControlType.kPosition));
     
  }

  public Command resetZeroCommand() {
    return
    startEnd(() -> {
      Utils.enableSoftLimits(m_armMotorLeft, false);
      Utils.enableSoftLimits(m_armMotorRight, false);
      m_armMotorLeft.set(-0.1);
    }, () -> {
      m_armLeftEncoder.setPosition(0);
      m_armRightEncoder.setPosition(0);
      m_armMotorLeft.set(0.0);
      Utils.enableSoftLimits(m_armMotorLeft, true);
      Utils.enableSoftLimits(m_armMotorRight, true);
      m_hasInitialZeroReset = true;
    })
    .withName("ResetClimb");
  }

  public boolean hasInitialZeroReset() {
    return m_hasInitialZeroReset;
  }

  public void reset() {
    m_armMotorLeft.set(0.0);
    m_armMotorRight.set(0.0);

    m_isBrakeApplied = false;
    m_isBeamBreakTriggered = false;

    m_brakeServo.setPosition(1.0);

    m_armLeftPIDController.setReference(Constants.Climber.kArmPositionStarting, ControlType.kPosition);
  }

  private void updateTelemetry() {
    double armLeftPosition = m_armLeftEncoder.getPosition();
    SmartDashboard.putNumber("Robot/Climber/ArmLeft/Position", armLeftPosition);

    double armRightPosition = m_armRightEncoder.getPosition();
    SmartDashboard.putNumber("Robot/Climber/ArmRight/Position", armRightPosition);

    SmartDashboard.putNumber("Robot/Climber/Servo/Position", m_brakeServo.getPosition());
    SmartDashboard.putNumber("Robot/Climber/Servo/Angle", m_brakeServo.getAngle());
    SmartDashboard.putNumber("Robot/Climber/Servo/Speed", m_brakeServo.getSpeed());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
