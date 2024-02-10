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
    startEnd(
      () -> {
        runTopBelts(MotorDirection.Forward); // run top belts for ground intake at front of robot
        runBottomBelts(MotorDirection.Forward); // run bottom belts for ground intake at front of robot
        runRollers(MotorDirection.Reverse); // run rollers for ground intake at front of robot
      },
      () -> {}
    )
    .unless(intakeHasTarget::get) // skip the command step if there is a note already in the intake
    .until(intakeHasTarget::get) // run this command step until there is a note in the intake
    .andThen(
      startEnd(
        () -> {
          runTopBelts(MotorDirection.Forward); // reverse the top belts to move the note from the intake to the launcher
          runBottomBelts(MotorDirection.Reverse);
          runRollers(MotorDirection.Forward); // reverse the rollers to push out any others notes from the front of the robot
        },
        () -> {}
      )
      .unless(launcherHasTarget::get) // skip this command step if there is a note already in the launcher
      .until(launcherHasTarget::get) // run this command step until there is a note in the launcher
    )
    .andThen(
      runOnce(
        () -> {
          runTopBelts(MotorDirection.None); // stop the top belts once a note has reached the launcher
          runBottomBelts(MotorDirection.None); // stop the bottom belts once a note has reached the launcher
          runRollers(MotorDirection.Reverse); // run the rollers to push out any other notes from the rear of the robot and assuming the direction of travel is now towards driver station for scoring
        }
      )
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None); // stop the top belts at any point the command sequence is ended or interrupted
      runBottomBelts(MotorDirection.None); // stop the bottom belts at any point the command sequence is ended or interrupted
      runRollers(MotorDirection.None); // stop the rollers at any point the command sequence is ended or interrupted
    })
    .withName("RunIntakeFromFrontCommand");
  }

  public Command runIntakeFromRearCommand(Supplier<Boolean> intakeHasTarget, Supplier<Boolean> launcherHasTarget) {
    return    
    startEnd(
      () -> {
        runTopBelts(MotorDirection.Reverse); // run top belts for ground intake at rear of robot
        runBottomBelts(MotorDirection.Reverse); // run bottom belts for ground intake at rear of robot
        runRollers(MotorDirection.Forward); // run rollers for ground intake at rear of robot
      },
      () -> {}
    )
    .unless(intakeHasTarget::get) // skip the command step if there is a note already in the intake
    .until(intakeHasTarget::get) // run this command step until there is a note in the intake
    .andThen(
      startEnd(
        () -> {
          runRollers(MotorDirection.Reverse); // reverse the rollers to push out any others notes from the rear of the robot
        },
        () -> {}
      )
      .unless(launcherHasTarget::get) // skip this command step if there is a note already in the launcher
      .until(launcherHasTarget::get) // run this command step until there is a note in the launcher
    )
    .andThen(
      runOnce(
        () -> {
          runTopBelts(MotorDirection.None); // stop the top belts once a note has reached the launcher
          runBottomBelts(MotorDirection.None); // stop the bottom belts once a note has reached the launcher
          runRollers(MotorDirection.Reverse); // run rollers to push out any others notes from the rear of the robot
        }
      )
    )
    .finallyDo(() -> {
      runTopBelts(MotorDirection.None); // stop the top belts at any point the command sequence is ended or interrupted
      runBottomBelts(MotorDirection.None); // stop the bottom belts at any point the command sequence is ended or interrupted
      runRollers(MotorDirection.None); // stop the rollers at any point the command sequence is ended or interrupted
    })
    .withName("RunIntakeFromRearCommand");
  }

  public Command runIntakeEjectCommand() {
    return
    startEnd(
      () -> {
        runTopBelts(MotorDirection.Forward);
        runBottomBelts(MotorDirection.Forward);
        runRollers(MotorDirection.Reverse);
      },
      () -> {
        runTopBelts(MotorDirection.None);
        runBottomBelts(MotorDirection.None);
        runRollers(MotorDirection.None);
      }
    )
    .withName("RunIntakeEjectCommand");
  }

  public Command runIntakeForLaunchCommand() {
    return
    startEnd(
      () -> {
        runTopBelts(MotorDirection.Forward); // run top belts to push note into launch rollers
        runBottomBelts(MotorDirection.None); // run bottom belts to push note into launch rollers
      },
      () -> {
        runTopBelts(MotorDirection.None);
        runBottomBelts(MotorDirection.None);
      }
    )
    .withName("RunIntakeForLaunchCommand");
  }

  private void runTopBelts(MotorDirection motorDirection) {
    switch (motorDirection) {
      case Forward:
        m_topBeltMotor.set(Constants.Intake.kTopBeltMotorMaxOutput);
        break;
      case Reverse:
        m_topBeltMotor.set(Constants.Intake.kTopBeltMotorMinOutput);
        break;
      case None:
        m_topBeltMotor.set(0.0);
        break;
      default:
        break;
    }
  }

  private void runBottomBelts(MotorDirection motorDirection) {
    switch (motorDirection) {
      case Forward:
        m_bottomBeltMotor.set(Constants.Intake.kTopBeltMotorMaxOutput);
        break;
      case Reverse:
        m_bottomBeltMotor.set(Constants.Intake.kTopBeltMotorMinOutput);
        break;
      case None:
        m_bottomBeltMotor.set(0.0);
        break;
      default:
        break;
    }
  }

  private void runRollers(MotorDirection motorDirection) {
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
