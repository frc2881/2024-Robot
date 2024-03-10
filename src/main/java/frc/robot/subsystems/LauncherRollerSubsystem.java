package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.common.Utils;

public class LauncherRollerSubsystem extends SubsystemBase {
  private final CANSparkFlex m_topRollerMotor;
  private final CANSparkFlex m_bottomRollerMotor;

  private double[] m_distances;
  private double[] m_speeds;

  public LauncherRollerSubsystem() {
    m_topRollerMotor = new CANSparkFlex(Constants.Launcher.kTopRollerMotorCANId, MotorType.kBrushless);
    m_topRollerMotor.restoreFactoryDefaults();
    m_topRollerMotor.setIdleMode(Constants.Launcher.kTopRollerMotorIdleMode); 
    m_topRollerMotor.setSmartCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);
    m_topRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);
    m_topRollerMotor.setInverted(true);
    m_topRollerMotor.burnFlash();

    m_bottomRollerMotor = new CANSparkFlex(Constants.Launcher.kBottomRollerMotorCANId, MotorType.kBrushless);
    m_bottomRollerMotor.restoreFactoryDefaults();
    m_bottomRollerMotor.setIdleMode(Constants.Launcher.kBottomRollerMotorIdleMode); 
    m_bottomRollerMotor.setSmartCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit);
    m_bottomRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit);
    m_bottomRollerMotor.burnFlash();

    m_distances = new double[Constants.Launcher.kRollerSpeeds.length];
    m_speeds = new double[Constants.Launcher.kRollerSpeeds.length];
    for (int i = 0; i < Constants.Launcher.kRollerSpeeds.length; i++) {
      m_distances[i] = Constants.Launcher.kRollerSpeeds[i].distance();
      m_speeds[i] = Constants.Launcher.kRollerSpeeds[i].speed();
    }  
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runCommand(Supplier<LauncherRollerSpeeds> rollerSpeeds) {
    return 
    startEnd(() -> {
      m_topRollerMotor.set(rollerSpeeds.get().top() * Constants.Launcher.kTopRollerMotorMaxOutput);
      m_bottomRollerMotor.set(rollerSpeeds.get().bottom() * Constants.Launcher.kBottomRollerMotorMaxOutput);
    },
    () -> { 
      m_topRollerMotor.set(0.0);
      m_bottomRollerMotor.set(0.0);
    })
    .withName("RunLauncherRollers");
  }

  public LauncherRollerSpeeds getSpeedsForArmPosition(Supplier<Double> distance) {
    double speed = Utils.getLinearInterpolation(m_distances, m_speeds, distance.get());
    SmartDashboard.putNumber("CalculatedLauncherSpeed", speed);

    double result = Utils.isValueBetween(speed, 0.0, Constants.Launcher.kArmMotorMaxOutput) 
    ? speed 
    : 0.8;
    return new LauncherRollerSpeeds(result, result);
    
    // TODO: add if to see if too far then move speed down 
  }

  public void reset() {
    m_topRollerMotor.set(0.0);
    m_bottomRollerMotor.set(0.0);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Launcher/Roller/Top/Speed", m_topRollerMotor.get());
    SmartDashboard.putNumber("Robot/Launcher/Roller/Bottom/Speed", m_bottomRollerMotor.get());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
