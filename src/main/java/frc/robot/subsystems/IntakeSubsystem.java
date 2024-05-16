package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  public Command runIntakeCommand(Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward, Constants.Intake.kIntakeBeltSpeeds); 
      runBottomBelts(MotorDirection.Forward, Constants.Intake.kIntakeBeltSpeeds);
      runRollers(MotorDirection.Reverse);
    }, () -> {})
    .onlyWhile(() -> !launcherBottomHasTarget.get())
    .andThen(
      new WaitCommand(Constants.Intake.kIntakeCompletionDelay)
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("RunIntake");
  }

  public Command runIntakeAutoCommand(Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward, 0.74); 
      runBottomBelts(MotorDirection.Forward, 0.74);
      runRollers(MotorDirection.Reverse);
    }, () -> {})
    .onlyWhile(() -> !launcherBottomHasTarget.get())
    .andThen(
      new WaitCommand(0.0193)
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("RunIntakeAuto");
  }

  public Command runAdjustNotePositionCommand(Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return Commands.run(
      () -> {
        runTopBelts(MotorDirection.Reverse, 0.3);
      }
    )
    .until(() -> !launcherTopHasTarget.get())
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
      runRollers(MotorDirection.None);
    })
    .withName("AdjustNotePosition");
  }

  public Command runEjectCommand() {
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
    .withName("RunEject");
  }

  public Command runLaunchCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward); 
      runBottomBelts(MotorDirection.Forward);  
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
        m_topBeltMotor.set(speed * Constants.Intake.kTopBeltMotorMaxForwardOutput);
        break;
      case Reverse:
        m_topBeltMotor.set(speed * Constants.Intake.kTopBeltMotorMaxReverseOutput);
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
        m_bottomBeltMotor.set(speed * Constants.Intake.kBottomBeltMotorMaxForwardOutput);
        break;
      case Reverse:
        m_bottomBeltMotor.set(speed * Constants.Intake.kBottomBeltMotorMaxReverseOutput);
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
        m_rollerMotor.set(Constants.Intake.kTopBeltMotorMaxForwardOutput);
        break;
      case Reverse:
        m_rollerMotor.set(Constants.Intake.kTopBeltMotorMaxReverseOutput);
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
}
