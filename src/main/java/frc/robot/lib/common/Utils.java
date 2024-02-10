package frc.robot.lib.common;

import com.fasterxml.jackson.databind.ObjectMapper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

public final class Utils {

  private static final ObjectMapper objectMapper = new ObjectMapper();

  public static String objectToJson(Object o) {
    try {
      return objectMapper.writeValueAsString(o);
    } catch (Exception ex) {
      return "{}";
    }
  }

  public static boolean isValueBetween(double value, double minValue, double maxValue) {
    return value >= minValue && value <= maxValue;
  }

  public static boolean isPoseInBounds(Pose2d value, Pose2d minPose, Pose2d maxPose) {
    return isValueBetween(value.getX(), minPose.getX(), maxPose.getX()) && isValueBetween(value.getY(), minPose.getY(), maxPose.getY());
  }

  public static double voltsToPsi(double sensorVoltage, double supplyVoltage) {
    return 250 * (sensorVoltage / supplyVoltage) - 25;
  }

  public static double squareInput(double input, double deadband) {
    double deadbandInput = MathUtil.applyDeadband(input, deadband);
    return (deadbandInput * deadbandInput) * Math.signum(input);
  }

  public static void enableSoftLimits(CANSparkBase controller, boolean isEnabled) {
    controller.enableSoftLimit(SoftLimitDirection.kForward, isEnabled);
    controller.enableSoftLimit(SoftLimitDirection.kReverse, isEnabled);
  }
}
