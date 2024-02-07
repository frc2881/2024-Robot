package frc.robot.lib.common;

public class PIDConstants {
  public final double P;
  public final double I;
  public final double D;
  public final double FF;

  public PIDConstants(double P, double I, double D, double FF) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.FF = FF;
  }

  public PIDConstants(double P, double I, double D) {
    this(P, I, D, 0.0);
  }
}

