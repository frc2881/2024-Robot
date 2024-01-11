package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.lib.sensors.Gyro;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution;
  private final Gyro m_gyro;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final XboxController m_driverController;
  private final XboxController m_operatorController;
  private final AutoCommands m_autoCommands;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
    m_gyro = new Gyro(Constants.Gyro.kIMUAxisYaw, Constants.Gyro.kIMUAxisPitch, Constants.Gyro.kIMUAxisRoll, Constants.Gyro.kSPIPort, Constants.Gyro.kCalibrationTime);
    m_driveSubsystem = new DriveSubsystem(m_gyro);
    m_poseSubsystem = new PoseSubsystem(m_gyro::getRotation2d, m_driveSubsystem::getSwerveModulePositions);

    m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);
    m_operatorController = new XboxController(Constants.Controllers.kOperatorControllerPort);
    setupControls();

    AutoBuilder.configureHolonomic(
      m_poseSubsystem::getPose, 
      m_poseSubsystem::resetPose, 
      m_driveSubsystem::getSpeeds, 
      m_driveSubsystem::drive, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Drive.kPathFollowerTranslationP, Constants.Drive.kPathFollowerTranslationI, Constants.Drive.kPathFollowerTranslationD),
        new PIDConstants(Constants.Drive.kPathFollowerRotationP, Constants.Drive.kPathFollowerRotationI, Constants.Drive.kPathFollowerRotationD), 
        Constants.Drive.kMaxSpeedMetersPerSecond, 
        Constants.Drive.kDriveBaseRadius, 
        new ReplanningConfig()
      ),
      () -> Robot.getAlliance() == Alliance.Red,
      m_driveSubsystem
    );
    m_autoCommands = new AutoCommands(m_driveSubsystem, m_poseSubsystem);
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

    new Trigger(m_driverController::getStartButton)
      .onTrue(m_driveSubsystem.setForCalibrationCommand());

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

    SendableChooser<DriveSubsystem.DriftCorrection> driveDriftCorrectionChooser = new SendableChooser<DriveSubsystem.DriftCorrection>();
    driveDriftCorrectionChooser.setDefaultOption(DriveSubsystem.DriftCorrection.ENABLED.toString(), DriveSubsystem.DriftCorrection.ENABLED);
    driveDriftCorrectionChooser.addOption(DriveSubsystem.DriftCorrection.DISABLED.toString(), DriveSubsystem.DriftCorrection.DISABLED);
    driveDriftCorrectionChooser.onChange(driftCorrection -> m_driveSubsystem.setDriftCorrection(driftCorrection));
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driveDriftCorrectionChooser);
  }

  private void setupAutos() {
    m_autoChooser.setDefaultOption("None", Commands.none());

    m_autoChooser.addOption("Test- Path", m_autoCommands.testPath());
    m_autoChooser.addOption("Test- Auto", m_autoCommands.testAuto());
    
    SmartDashboard.putData("Robot/Auto/Command", m_autoChooser);
  }

  public Command getSelectedAutoCommand() {
    return m_autoChooser.getSelected();
  }

  public void reset() {
    m_driveSubsystem.reset();
    m_poseSubsystem.reset();
  }
}
