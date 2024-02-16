package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Enums.MotorDirection;

public class IntakeSubsystem extends SubsystemBase {
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

  public Command runIntakeFromFrontCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherHasTarget) {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward);
      runBottomBelts(MotorDirection.Forward);
      runRollers(MotorDirection.Reverse);
    }, () -> {})
    .onlyWhile(() -> !intakeHasTarget.get())
    .andThen(
      startEnd(() -> {
        runTopBelts(MotorDirection.Forward, 0.5);
        runBottomBelts(MotorDirection.Reverse, 0.5);
        runRollers(MotorDirection.Forward);
      }, () -> {})
      .onlyWhile(() -> !launcherHasTarget.get())
    )
    .andThen(
      runOnce(() -> {
        runTopBelts(MotorDirection.None);
        runBottomBelts(MotorDirection.None);
        runRollers(MotorDirection.Reverse);
      })
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("RunIntakeFromFront");
  }

  public Command runIntakeFromRearCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherHasTarget) {
    return    
    startEnd(() -> {
      runTopBelts(MotorDirection.Reverse);
      runBottomBelts(MotorDirection.Reverse);
      runRollers(MotorDirection.Forward);
    }, () -> {})
    .onlyWhile(() -> !intakeHasTarget.get())
    .andThen(
      startEnd(() -> {
        runRollers(MotorDirection.Reverse);
      }, () -> {})
      .onlyWhile(() -> !launcherHasTarget.get())
    )
    .andThen(
      runOnce(() -> {
        runTopBelts(MotorDirection.None); 
        runBottomBelts(MotorDirection.None); 
        runRollers(MotorDirection.Reverse); 
      })
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None); 
      runRollers(MotorDirection.None); 
    })
    .withName("RunIntakeFromRear");
  }

  public Command runIntakeEjectCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward);
      runBottomBelts(MotorDirection.Forward);
      runRollers(MotorDirection.Reverse);
    }, () -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("RunIntakeEject");
  }

  public Command runIntakeForLaunchPositionCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Reverse, 0.15); 
      runBottomBelts(MotorDirection.Reverse, 0.15); 
    }, () -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
    })
    .withName("RunIntakeForLaunchPosition");
  }

  // TODO: experiment with running bottom belts at slow speed during launch to help note into rollers
  public Command runIntakeForLaunchCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward); 
      runBottomBelts(MotorDirection.Reverse, 0.25);  
    }, () -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
    })
    .withName("RunIntakeForLaunch");
  }

  private void runTopBelts(MotorDirection motorDirection) {
    runTopBelts(motorDirection, 1.0);
  }

  private void runTopBelts(MotorDirection motorDirection, Double speed) {
    switch (motorDirection) {
      case Forward:
        m_topBeltMotor.set(speed * Constants.Intake.kTopBeltMotorMaxOutput);
        break;
      case Reverse:
        m_topBeltMotor.set(speed * Constants.Intake.kTopBeltMotorMinOutput);
        break;
      case None:
        m_topBeltMotor.set(0.0);
        break;
      default:
        break;
    }
  }

  private void runBottomBelts(MotorDirection motorDirection) {
    runBottomBelts(motorDirection, 1.0);
  }

  private void runBottomBelts(MotorDirection motorDirection, Double speed) {
    switch (motorDirection) {
      case Forward:
        m_bottomBeltMotor.set(speed * Constants.Intake.kBottomBeltMotorMaxOutput);
        break;
      case Reverse:
        m_bottomBeltMotor.set(speed * Constants.Intake.kBottomBeltMotorMinOutput);
        break;
      case None:
        m_bottomBeltMotor.set(0.0);
        break;
      default:
        break;
    }
  }

  private void runRollers(MotorDirection motorDirection) {
    runRollers(motorDirection, 1.0);
  }

  private void runRollers(MotorDirection motorDirection, Double speed) {
    switch (motorDirection) {
      case Forward:
        m_rollerMotor.set(Constants.Intake.kTopBeltMotorMaxOutput);
        break;
      case Reverse:
        m_rollerMotor.set(Constants.Intake.kTopBeltMotorMinOutput);
        break;
      case None:
        m_rollerMotor.set(0.0);
        break;
      default:
        break;
    }
  }

  private void updateTelemetry() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
