package frc.robot.lib.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.Utils;

public class Gyro extends ADIS16470_IMU {

  private double m_angleAdjustment = 0.0;

  public Gyro(IMUAxis imuAxis, SPI.Port port, CalibrationTime calibrationTime) {
    super(imuAxis, port, calibrationTime);
  }

  @Override
  public void reset() {
    reset(0);
  }

  public void reset(double value) {
    m_angleAdjustment = value;
    super.reset();
  }

  public Command resetCommand() {
    return Commands.runOnce(
      () -> this.reset(0))
      .ignoringDisable(true)
      .withName("ResetGyro");
  }

  public double getRoll() {
    return super.getYComplementaryAngle();
  }

  public double getPitch() {
    return super.getXComplementaryAngle();
  }

  public double getYaw() {
    return getAngle();
  }

  @Override
  public double getAngle() {
    return super.getAngle() + m_angleAdjustment;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public Rotation3d getRotation3d() {
    return new Rotation3d(
      Units.degreesToRadians(getRoll()), 
      Units.degreesToRadians(getPitch()),
      Units.degreesToRadians(getYaw())
    );
  }

  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return super.getRate();
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Gyro/Roll", getRoll());
    SmartDashboard.putNumber("Robot/Gyro/Pitch", getPitch());
    SmartDashboard.putNumber("Robot/Gyro/Yaw", getYaw());
    SmartDashboard.putNumber("Robot/Gyro/Heading", getHeading());
    SmartDashboard.putNumber("Robot/Gyro/TurnRate", getTurnRate());
    SmartDashboard.putString("Robot/Gyro/Rotation2d", Utils.objectToJson(getRotation2d()));
    SmartDashboard.putString("Robot/Gyro/Rotation3d", Utils.objectToJson(getRotation3d()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Roll", this::getRoll, null);
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    builder.addDoubleProperty("Heading", this::getHeading, null);
    builder.addDoubleProperty("TurnRate", this::getTurnRate, null);
  }
}
