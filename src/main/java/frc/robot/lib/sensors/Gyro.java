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

  public Gyro(
    IMUAxis imuAxisYaw, 
    IMUAxis imuAxisPitch, 
    IMUAxis imuAxisRoll, 
    SPI.Port port, 
    CalibrationTime calibrationTime
  ) {
    super(imuAxisYaw, imuAxisPitch, imuAxisRoll, port, calibrationTime);
  }

  @Override
  public void reset() {
    reset(0);
  }

  public void reset(double value) {
    setGyroAngle(getYawAxis(), value);
  }

  public Command resetCommand() {
    return Commands.runOnce(
      () -> this.reset(0))
      .ignoringDisable(true)
      .withName("ResetGyro");
  }

  public Command resetCommand(double value) {
    return Commands.runOnce(
      () -> this.reset(value))
      .ignoringDisable(true)
      .withName("ResetGyro");
  }

  public double getRoll() {
    return getAngle(getRollAxis());
  }

  public double getPitch() {
    return getAngle(getPitchAxis());
  }

  public double getYaw() {
    return getAngle(getYawAxis());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
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
    return getRate(getYawAxis());
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
