package frc.robot.utils;

import edu.wpi.first.wpilibj.I2C;

public class sensors {
  public static BNO055 gyro;

  public static void initAllSensors() {
    gyro = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_NDOF_FMC_OFF, BNO055.vector_type_t.VECTOR_EULER,
      I2C.Port.kMXP, BNO055.BNO055_ADDRESS_A
    );
  }

  public static void initGyro() {
    gyro = BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_NDOF_FMC_OFF, BNO055.vector_type_t.VECTOR_EULER,
      I2C.Port.kMXP, BNO055.BNO055_ADDRESS_A
    );
  }
}
