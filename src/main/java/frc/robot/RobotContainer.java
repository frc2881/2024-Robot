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
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.lib.sensors.ObjectSensor;
import frc.robot.lib.logging.Logger;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution;
  private final GyroSensor m_gyroSensor;
  private final ObjectSensor m_objectSensor;
  private final DistanceSensor m_intakeDistanceSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  // private final ArmSubsystem m_armSubsystem;
  // private final IntakeSubsystem m_intakeSubsystem;
  // private final LauncherSubsystem m_launcherSubsystem;
  // private final PickupSubsystem m_pickupSubsystem;

  private final XboxController m_driverController;
  private final XboxController m_operatorController;

  private final AutoCommands m_autoCommands;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    m_gyroSensor = new GyroSensor(Constants.Sensors.Gyro.kIMUAxisYaw, Constants.Sensors.Gyro.kIMUAxisPitch, Constants.Sensors.Gyro.kIMUAxisRoll, Constants.Sensors.Gyro.kSPIPort, Constants.Sensors.Gyro.kCalibrationTime);
    m_objectSensor = new ObjectSensor(Constants.Sensors.Object.kCameraName);
    m_intakeDistanceSensor = new DistanceSensor(Constants.Sensors.Distance.kIntakeSensorName);
    
    m_driveSubsystem = new DriveSubsystem(m_gyroSensor);
    m_poseSubsystem = new PoseSubsystem(m_gyroSensor, m_driveSubsystem::getSwerveModulePositions);
    // m_armSubsystem = new ArmSubsystem();
    // m_intakeSubsystem = new IntakeSubsystem();
    // m_launcherSubsystem = new LauncherSubsystem();
    // m_pickupSubsystem = new PickupSubsystem();

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
    m_autoCommands = new AutoCommands(m_driveSubsystem, m_poseSubsystem, m_gyroSensor);
    m_autoChooser = new SendableChooser<Command>();
    setupAutos();
  }

  private void setupControls() {

    // DRIVER =========================

    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithControllerCommand(m_driverController));
    //m_launcherSubsystem.setDefaultCommand(m_launcherSubsystem.updateLaunchAngle(m_poseSubsystem::getPose));

    new Trigger(m_driverController::getXButton)
      .onTrue(m_driveSubsystem.toggleLockStateCommand());

    new Trigger(m_driverController::getRightStickButton)
      .onFalse(m_gyroSensor.resetCommand());

    // OPERATOR =========================

    // new Trigger(() -> Math.abs(m_operatorController.getLeftY()) > 0.1)
    //   .whileTrue(m_armSubsystem.runLeadScrewCommand(m_operatorController::getLeftY));

    // new Trigger(() -> Math.abs(m_operatorController.getRightY()) > 0.1)
    //   .whileTrue(m_launcherSubsystem.runLeadScrewCommand(m_operatorController::getRightY));

    // DASHBOARD =========================

    SendableChooser<DriveSubsystem.SpeedMode> driveSpeedModeChooser = new SendableChooser<DriveSubsystem.SpeedMode>();
    driveSpeedModeChooser.setDefaultOption(DriveSubsystem.SpeedMode.Competition.toString(), DriveSubsystem.SpeedMode.Competition);
    driveSpeedModeChooser.addOption(DriveSubsystem.SpeedMode.Training.toString(), DriveSubsystem.SpeedMode.Training);
    driveSpeedModeChooser.onChange(speedMode -> m_driveSubsystem.setSpeedMode(speedMode));
    SmartDashboard.putData("Robot/Drive/SpeedMode", driveSpeedModeChooser);

    SendableChooser<DriveSubsystem.Orientation> driveOrientationChooser = new SendableChooser<DriveSubsystem.Orientation>();
    driveOrientationChooser.setDefaultOption(DriveSubsystem.Orientation.Field.toString(), DriveSubsystem.Orientation.Field);
    driveOrientationChooser.addOption(DriveSubsystem.Orientation.Robot.toString(), DriveSubsystem.Orientation.Robot);
    driveOrientationChooser.onChange(orientation -> m_driveSubsystem.setOrientation(orientation));
    SmartDashboard.putData("Robot/Drive/Orientation", driveOrientationChooser);

    SendableChooser<DriveSubsystem.DriftCorrection> driveDriftCorrectionChooser = new SendableChooser<DriveSubsystem.DriftCorrection>();
    driveDriftCorrectionChooser.setDefaultOption(DriveSubsystem.DriftCorrection.Enabled.toString(), DriveSubsystem.DriftCorrection.Enabled);
    driveDriftCorrectionChooser.addOption(DriveSubsystem.DriftCorrection.Disabled.toString(), DriveSubsystem.DriftCorrection.Disabled);
    driveDriftCorrectionChooser.onChange(driftCorrection -> m_driveSubsystem.setDriftCorrection(driftCorrection));
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driveDriftCorrectionChooser);
  }

  private void setupAutos() {
    m_autoChooser.setDefaultOption("None", Commands.none());

    m_autoChooser.addOption("Test- 3 Note Auto", m_autoCommands.test3NoteAuto());
    m_autoChooser.addOption("Test- Path", m_autoCommands.testPath2());
    
    SmartDashboard.putData("Robot/Auto/Command", m_autoChooser);
  }

  public Command getSelectedAutoCommand() {
    return m_autoChooser.getSelected();
  }

  public void updateTelemetry() {
    m_gyroSensor.updateTelemetry();
    m_objectSensor.updateTelemetry();
  }
}
