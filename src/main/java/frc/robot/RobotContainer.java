package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoCommands;
import frc.robot.lib.logging.Logger;
import frc.robot.lib.sensors.Gyro;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution;
  private final Gyro m_gyro;
  private final DriveSubsystem m_driveSubsystem;
  private final XboxController m_driverController;
  private final XboxController m_operatorController;
  private final AutoCommands m_autoCommands;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
    m_gyro = new Gyro(Constants.Gyro.kIMUAxis, Constants.Gyro.kSPIPort, Constants.Gyro.kCalibrationTime);
    m_driveSubsystem = new DriveSubsystem(m_gyro);

    m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);
    m_operatorController = new XboxController(Constants.Controllers.kOperatorControllerPort);
    setupControls();

    m_autoCommands = new AutoCommands(m_driveSubsystem);
    m_autoChooser = new SendableChooser<Command>();
    setupAutos();
  }

  private void setupControls() {

    // DRIVER =========================

    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithControllerCommand(m_driverController));

    new Trigger(m_driverController::getXButton)
      .onTrue(m_driveSubsystem.toggleLockStateCommand());

    new Trigger(m_driverController::getRightStickButton)
      .onFalse(m_gyro.resetCommand());

    // OPERATOR =========================


    // DASHBOARD =========================

    SendableChooser<DriveSubsystem.SpeedMode> driveSpeedModeChooser = new SendableChooser<DriveSubsystem.SpeedMode>();
    driveSpeedModeChooser.setDefaultOption(DriveSubsystem.SpeedMode.COMPETITION.toString(), DriveSubsystem.SpeedMode.COMPETITION);
    driveSpeedModeChooser.addOption(DriveSubsystem.SpeedMode.TRAINING.toString(), DriveSubsystem.SpeedMode.TRAINING);
    driveSpeedModeChooser.onChange(speedMode -> m_driveSubsystem.setSpeedMode(speedMode));
    SmartDashboard.putData("Robot/Drive/SpeedMode", driveSpeedModeChooser);

    SendableChooser<DriveSubsystem.Orientation> driveOrientationChooser = new SendableChooser<DriveSubsystem.Orientation>();
    driveOrientationChooser.setDefaultOption(DriveSubsystem.Orientation.FIELD.toString(), DriveSubsystem.Orientation.FIELD);
    driveOrientationChooser.addOption(DriveSubsystem.Orientation.ROBOT.toString(), DriveSubsystem.Orientation.ROBOT);
    driveOrientationChooser.onChange(orientation -> m_driveSubsystem.setOrientation(orientation));
    SmartDashboard.putData("Robot/Drive/Orientation", driveOrientationChooser);
  }

  private void setupAutos() {
    m_autoChooser.setDefaultOption("None", Commands.none());

    m_autoChooser.addOption("Test- Path", m_autoCommands.testPath());
    m_autoChooser.addOption("Test- Auto", m_autoCommands.testAuto());
    
    SmartDashboard.putData("Robot/Auto/Command", m_autoChooser);
  }

  public Command getSelectedAutoCommand() {
    Command command = m_autoChooser.getSelected();
    Logger.log("Auto command selected:" + command.getName());
    return command;
  }

  public void resetRobot() {
    m_driveSubsystem.resetEncoders();
  }
}
