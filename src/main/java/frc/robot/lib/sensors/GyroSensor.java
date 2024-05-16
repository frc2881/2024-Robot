package frc.robot.lib.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Utils;
import frc.robot.lib.common.Enums.RobotState;

public class GyroSensor extends ADIS16470_IMU {

  public GyroSensor(
    IMUAxis imuAxisYaw, 
    IMUAxis imuAxisPitch, 
    IMUAxis imuAxisRoll, 
    SPI.Port port, 
    CalibrationTime calibrationTime
  ) {
    super(imuAxisYaw, imuAxisPitch, imuAxisRoll, port, calibrationTime);
  }

  public Command calibrateShortCommand() {
    return Commands.runOnce(
    () -> {
      SmartDashboard.putBoolean("Robot/Sensor/Gyro/IsCalibrating", true);
      configCalTime(CalibrationTime._4s);
      calibrate();
    })
    .andThen(
      new WaitCommand(4.0)
    )
    .andThen(Commands.runOnce(
      () -> {
        SmartDashboard.putBoolean("Robot/Sensor/Gyro/IsCalibrating", false);
      })
    )
    .onlyIf(() -> Robot.getState() == RobotState.Disabled)
    .ignoringDisable(true)
    .withName("CalibrateGyroShort");
  }

  public Command calibrateLongCommand() {
    return Commands.runOnce(
    () -> {
      SmartDashboard.putBoolean("Robot/Sensor/Gyro/IsCalibrating", true);
      configCalTime(Constants.Sensors.Gyro.kCalibrationTime);
      calibrate();
    })
    .andThen(
      new WaitCommand(8.0)
    )
    .andThen(Commands.runOnce(
      () -> {
        SmartDashboard.putBoolean("Robot/Sensor/Gyro/IsCalibrating", false);
      })
    )
    .onlyIf(() -> Robot.getState() == RobotState.Disabled)
    .ignoringDisable(true)
    .withName("CalibrateGyroLong");
  }

  @Override
  public void reset() {
    reset(0);
  }

  public void reset(double value) {
    setGyroAngle(getYawAxis(), value);
  }

  public void resetRobotToField(Pose2d robotPose) {
    reset(Utils.wrapAngle(robotPose.getRotation().getDegrees() + Utils.getValueForAlliance(0, 180)));
  }

  public Command resetCommand() {
    return Commands.runOnce(
    () -> reset(0))
    .ignoringDisable(true)
    .withName("ResetGyroToZero");
  }

  public Command resetCommand(double value) {
    return Commands.runOnce(
    () -> reset(value))
    .ignoringDisable(true)
    .withName("ResetGyroToAngle");
  }

  public double getRoll() {
    return Utils.wrapAngle(getAngle(getRollAxis()));
  }

  public double getPitch() {
    return Utils.wrapAngle(getAngle(getPitchAxis()));
  }

  public double getYaw() {
    return Utils.wrapAngle(getAngle(getYawAxis()));
  }

  public double getHeading() {
    return getYaw();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  public double getTurnRate() {
    return getRate(getYawAxis());
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Yaw", getYaw());
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Pitch", getPitch());
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Roll", getRoll());
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Heading", getHeading());
    SmartDashboard.putNumber("Robot/Sensor/Gyro/TurnRate", getTurnRate());
  }
}
