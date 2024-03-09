package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    m_topBeltMotor.burnFlash();

    m_bottomBeltMotor = new CANSparkMax(Constants.Intake.kBottomBeltMotorCANId, MotorType.kBrushless);
    m_bottomBeltMotor.restoreFactoryDefaults();
    m_bottomBeltMotor.setIdleMode(Constants.Intake.kBottomBeltMotorIdleMode); 
    m_bottomBeltMotor.setSmartCurrentLimit(Constants.Intake.kBottomBeltMotorCurrentLimit);
    m_bottomBeltMotor.setSecondaryCurrentLimit(Constants.Intake.kBottomBeltMotorCurrentLimit);
    m_bottomBeltMotor.burnFlash();

    m_rollerMotor = new CANSparkMax(Constants.Intake.kRollerMotorCANId, MotorType.kBrushless);
    m_rollerMotor.restoreFactoryDefaults();
    m_rollerMotor.setIdleMode(Constants.Intake.kRollerMotorIdleMode); 
    m_rollerMotor.setSmartCurrentLimit(Constants.Intake.kRollerMotorCurrentLimit);
    m_rollerMotor.setSecondaryCurrentLimit(Constants.Intake.kRollerMotorCurrentLimit);
    m_rollerMotor.burnFlash();
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runIntakeFrontCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward);
      runBottomBelts(MotorDirection.Forward);
      runRollers(MotorDirection.Reverse);
    }, () -> {})
    .onlyWhile(() -> !intakeHasTarget.get())
    .andThen(
      Commands.waitSeconds(0.14)
    )
    .andThen(
      startEnd(() -> {
        runTopBelts(MotorDirection.Forward, 0.4); 
        runBottomBelts(MotorDirection.Reverse, 0.4);
        runRollers(MotorDirection.Forward);
      }, () -> {})
      .onlyWhile(() -> !launcherBottomHasTarget.get())
      .andThen(
        Commands.waitSeconds(0.2)
      )
    )
    .andThen(
      runOnce(() -> {
        runTopBelts(MotorDirection.None);
        runRollers(MotorDirection.Reverse);
      })
      .andThen(
        run(() -> runBottomBelts(MotorDirection.Reverse)).withTimeout(0.4)
      )
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("RunIntakeFront");
  }

  public Command runIntakeRearCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return    
    startEnd(() -> {
      runTopBelts(MotorDirection.Reverse);
      runBottomBelts(MotorDirection.Reverse);
      runRollers(MotorDirection.Forward);
    }, () -> {})
    .onlyWhile(() -> !intakeHasTarget.get()) 
    .andThen(
      Commands.waitSeconds(0.05)
    )
    .andThen(
      startEnd(() -> {
        runTopBelts(MotorDirection.Forward, 0.4);
        runBottomBelts(MotorDirection.Reverse, 0.4);
        runRollers(MotorDirection.Forward);
      }, () -> {})
      .onlyWhile(() -> !launcherBottomHasTarget.get())
      .andThen(
        Commands.waitSeconds(0.2)
      )
    )
    .andThen(
      runOnce(() -> {
        runTopBelts(MotorDirection.None);
        runRollers(MotorDirection.Reverse);
      })
      .andThen(
        run(() -> runBottomBelts(MotorDirection.Reverse)).withTimeout(0.5)
      )
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None); 
      runRollers(MotorDirection.None); 
    })
    .withName("RunIntakeRear");
  }

  public Command runEjectFrontCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Reverse);
      runBottomBelts(MotorDirection.Reverse);
      runRollers(MotorDirection.Forward);
    }, () -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("RunEjectFront");
  }

  public Command runEjectRearCommand() {
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
    .withName("RunEjectRear");
  }

  public Command runLaunchCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward); 
      runBottomBelts(MotorDirection.Reverse, 0.5);  
    }, () -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
    })
    .withName("RunLaunch");
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

  public void reset() {
    m_topBeltMotor.set(0.0);
    m_bottomBeltMotor.set(0.0);
    m_rollerMotor.set(0.0);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Intake/Belt/Top/Speed", m_topBeltMotor.get());
    SmartDashboard.putNumber("Robot/Intake/Belt/Bottom/Speed", m_bottomBeltMotor.get());
    SmartDashboard.putNumber("Robot/Intake/Roller/Speed", m_rollerMotor.get());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
