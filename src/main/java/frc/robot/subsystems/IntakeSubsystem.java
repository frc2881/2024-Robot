package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
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

  public Command runIntakeFromFrontCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward);
      runBottomBelts(MotorDirection.Forward);
      runRollers(MotorDirection.Reverse);
    }, () -> {})
    .onlyWhile(() -> !intakeHasTarget.get())
    .andThen(
      new WaitCommand(0.14)
    )
    .andThen(
      startEnd(() -> {
        runTopBelts(MotorDirection.Forward, 0.4);
        runBottomBelts(MotorDirection.Reverse, 0.4);
        runRollers(MotorDirection.Forward);
      }, () -> {})
      .onlyWhile(() -> !launcherBottomHasTarget.get())
      .andThen(
      new WaitCommand(0.06)
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
    .withName("RunIntakeFromFront");
  }

  public Command runIntakeFromRearCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherTopHasTarget, Supplier<Boolean> launcherBottomHasTarget) {
    return    
    startEnd(() -> {
      runTopBelts(MotorDirection.Reverse);
      runBottomBelts(MotorDirection.Reverse);
      runRollers(MotorDirection.Forward);
    }, () -> {})
    .onlyWhile(() -> !intakeHasTarget.get()) 
    .andThen(
      new WaitCommand(0.05)
    )
    .andThen(
      startEnd(() -> {
        runTopBelts(MotorDirection.Forward, 0.4);
        runBottomBelts(MotorDirection.Reverse, 0.4);
        runRollers(MotorDirection.Forward);
      }, () -> {})
      .onlyWhile(() -> !launcherBottomHasTarget.get())
      .andThen(
      new WaitCommand(0.06)
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
    .withName("RunIntakeFromRear");
  }

  public Command runIntakeEjectRearCommand() {
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

  public Command runIntakeEjectFrontCommand() {
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
    .withName("RunIntakeEject");
  }



  public Command runIntakeForLaunchPositionCommand(Supplier<Double> distanceSupplier) {
    return
    startEnd(() -> {
      if(distanceSupplier.get() < 5.5){
        runTopBelts(MotorDirection.Reverse, 0.15); 
        runBottomBelts(MotorDirection.Reverse, 0.15); 
      }
      else if (distanceSupplier.get() > 10.0){
        runTopBelts(MotorDirection.Forward, 0.15); 
        runBottomBelts(MotorDirection.Forward, 0.15); 
      }
    }, () -> {
      runTopBelts(MotorDirection.None);
      runBottomBelts(MotorDirection.None);
    })
    .withName("RunIntakeForLaunchPosition");
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

  public Command runIntakeForLaunchCommand() {
    return
    startEnd(() -> {
      runTopBelts(MotorDirection.Forward); 
      runBottomBelts(MotorDirection.Reverse, 0.5);  
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
