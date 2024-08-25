package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int Player1Port = 0;

    public static final double kMove = 1;
    public static final double kTrun = 1;
  }

  //field constants
  public static class FieldConstants {}

  //robot mechanical constants
  public static class MechanicalConstants {
    public static final double RobotLength = 1.0;
    public static final double RobotWidth = 1.0;
    public static final double r = Math.sqrt(Math.pow(RobotLength ,2) + Math.pow(RobotWidth ,2));
  }

  //motor power constants
  public static class MotorPWRConstants {}

  //motor controller ID
  public static class MotorControllerID {
    public static final int LF_TurnID         = 1;
    public static final int LR_TurnID         = 4;
    public static final int RF_TurnID         = 2;
    public static final int RR_TurnID         = 3;

    public static final int LF_DriveID        = 5;
    public static final int LR_DriveID        = 8;
    public static final int RF_DriveID        = 6;
    public static final int RR_DriveID        = 7;
  }

  public static enum AutoActions {}
}
