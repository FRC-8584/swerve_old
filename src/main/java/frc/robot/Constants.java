package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class driveConstants {
    public static final double kx = 0.5;
    public static final double ky = 0.5;
    public static final double kturn = 0.5;

    public static final double length = 1;
    public static final double width = 1;
    public static final double r = Math.sqrt(length*length + width*width);
  }
}
