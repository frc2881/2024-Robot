package frc.robot.lib.common;

import com.fasterxml.jackson.databind.ObjectMapper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

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

  public static boolean isRobotInBounds(Pose2d robotPose, Pose2d minPose, Pose2d maxPose) {
    return isValueBetween(robotPose.getX(), minPose.getX(), maxPose.getX()) && isValueBetween(robotPose.getY(), minPose.getY(), maxPose.getY());
  }

  public static Rotation3d getTargetRotation(Pose2d robotPose, Pose3d targetPose) {
    Transform2d transform = robotPose.minus(targetPose.toPose2d());
    double yaw = new Rotation2d(transform.getX(), transform.getY()).getRadians();

    double height = targetPose.minus(new Pose3d(robotPose)).getZ();
    double distance = robotPose.getTranslation().getDistance(targetPose.toPose2d().getTranslation()); 
    double pitch = Math.atan2(height, distance);

    return new Rotation3d(0.0, pitch, yaw);
  }

  public static double squareInput(double input, double deadband) {
    double deadbandInput = MathUtil.applyDeadband(input, deadband);
    return (deadbandInput * deadbandInput) * Math.signum(input);
  }

  public static void enableSoftLimits(CANSparkBase controller, boolean isEnabled) {
    controller.enableSoftLimit(SoftLimitDirection.kForward, isEnabled);
    controller.enableSoftLimit(SoftLimitDirection.kReverse, isEnabled);
  }

  public static double voltsToPsi(double sensorVoltage, double supplyVoltage) {
    return 250 * (sensorVoltage / supplyVoltage) - 25;
  }
}
