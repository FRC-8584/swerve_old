package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.utils.tools;
import frc.robot.utils.PID;
import frc.robot.utils.sensors;

public class swerve{
  /**********swerve motor modules**********/

	public static final swerveModule lf = new swerveModule(1, 5);
  public static final swerveModule lr = new swerveModule(4, 8);
  public static final swerveModule rf = new swerveModule(2, 6);
  public static final swerveModule rr = new swerveModule(3, 7);

  /**********variables**********/

  private double robotHeading;
  private double driverHeading = 0;

  private boolean startMoving = true;

  /**********constants**********/

  private static final double length = 1, width = 1;
  private static final double r = Math.sqrt(length*length + width*width);
  private static final double kMove = 1;
  private static final double kTurn = 1;

  /**********functions**********/

  public void swerve() {
    lf.turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    lr.turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    rf.turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    rr.turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    lf.name = "Left_front";
    lr.name = "Left_rear";
    rf.name = "Right_front";
    rr.name = "Right_rear";

    lf.pid = new PID(1, 1e-3, 0);
    lr.pid = new PID(1, 1e-3, 0);
    rf.pid = new PID(1, 1e-3, 0);
    rr.pid = new PID(1, 1e-3, 0);

    move(0, 0, 0);
  }

  /**
   * Move the robot.
   * 
   * @param x : vector x force (driver heading)
   * @param y : vector y force (driver heading)
   * @param turn : turn force
   */
  public void move(double x, double y, double turn) {
    if(x == 0 && y == 0 && turn == 0){//doesn't move
      lf.stopMoving();
      lr.stopMoving();
      rf.stopMoving();
      rr.stopMoving();

      startMoving = true;

      return;
    }

    //convert the vector of driver's heading to the vector of robot's heading

    // final double tempVector[] = convertHeading(x, y);

    // x = tempVector[0];
    // y = tempVector[1];

    x *= kMove;
    y *= kMove;
    turn *= kTurn;

    //calculate vector (x, y)

    final double lf_x = x + turn*(length / r), lf_y = y + turn*(width / r);
    final double lr_x = x - turn*(length / r), lr_y = y + turn*(width / r);
    final double rf_x = x + turn*(length / r), rf_y = y - turn*(width / r);
    final double rr_x = x - turn*(length / r), rr_y = y - turn*(width / r);

    //calculate turn degrees

    final double lf_degrees = tools.toDegrees(lf_x, lf_y);
    final double lr_degrees = tools.toDegrees(lr_x, lr_y);
    final double rf_degrees = tools.toDegrees(rf_x, rf_y);
    final double rr_degrees = tools.toDegrees(rr_x, rr_y);

    //calculate motor speed

    double lf_speed = Math.sqrt(lf_x*lf_x + lf_y*lf_y);
    double lr_speed = Math.sqrt(lr_x*lr_x + lr_y*lr_y);
    double rf_speed = Math.sqrt(rf_x*rf_x + rf_y*rf_y);
    double rr_speed = Math.sqrt(rr_x*rr_x + rr_y*rr_y);

    //bounding

    double max = 1;
    if(lf_speed > max) max = lf_speed;
    if(lr_speed > max) max = lr_speed;
    if(rf_speed > max) max = rf_speed;
    if(rr_speed > max) max = rr_speed;

    lf_speed /= max;
    lr_speed /= max;
    rf_speed /= max;
    rr_speed /= max;

		//setpoint

    if(startMoving == true){
      if(lf.getTurnError() != 0 || lr.getTurnError() != 0 || rf.getTurnError() != 0 || rr.getTurnError() != 0){
        lf.setpoint(0, lf_degrees);
        lr.setpoint(0, lr_degrees);
        rf.setpoint(0, rf_degrees);
        rr.setpoint(0, rr_degrees);

        return;
      }
      else{
        startMoving = false;
      }
    }
    lf.setpoint(lf_speed, lf_degrees);
    lr.setpoint(lr_speed, lr_degrees);
    rf.setpoint(rf_speed, rf_degrees);
    rr.setpoint(rr_speed, rr_degrees);
		
    return;
  }

  /**
   * Get turnmotor encoder value.
   */
  public void getEncValue() {
    lf.getEncValue();
    lr.getEncValue();
    rf.getEncValue();
    rr.getEncValue();
  }

  /**
   * Get robot heading.
   * 
   * @return robot heading (0 =< value < 360 degrees)
   */
  public void getRobotHeading() {
    if(sensors.gyro.isInitialized()){
      robotHeading = sensors.gyro.getVector()[0];
    }
    else{
      robotHeading = driverHeading;
    }
  }

  /**
   * Change the vector of driver's heading to the vector of robot's heading.
   * 
   * @param x : vector x (driver heading)
   * @param y : vector y (driver heading)
   * 
   * @return a vector {x, y} (robot heading)
   */
  private double[] convertHeading(double x, double y) {
    return tools.toVector(
      Math.sqrt(x*x + y*y),
      tools.toDegrees(x, y) - (robotHeading - driverHeading)
    );//radius, angle
  }
}
