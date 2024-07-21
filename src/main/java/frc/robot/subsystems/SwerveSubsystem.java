package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule lf;
  private final SwerveModule lr;
  private final SwerveModule rf;
  private final SwerveModule rr;

  private double lf_degrees, lf_speed;
  private double lr_degrees, lr_speed;
  private double rf_degrees, rf_speed;
  private double rr_degrees, rr_speed;

  public SwerveSubsystem() {
    lf = new SwerveModule(1, 5, "left-front Swerve Module");
    lr = new SwerveModule(4, 8, "left-rear Swerve Module");
    rf = new SwerveModule(2, 6, "rigft-front Swerve Module");
    rr = new SwerveModule(3, 7, "rigft-rear Swerve Module");

    lf_degrees = 0; lf_speed = 0;
    lr_degrees = 0; lr_speed = 0;
    rf_degrees = 0; rf_speed = 0;
    rr_degrees = 0; rr_speed = 0;
  }

  @Override
  public void periodic() {
    lf.getEncValue();
    lr.getEncValue();
    rf.getEncValue();
    rr.getEncValue();

    lf.setpoint(lf_speed, lf_degrees);
    lr.setpoint(lr_speed, lr_degrees);
    rf.setpoint(rf_speed, rf_degrees);
    rr.setpoint(rr_speed, rr_degrees);
  }

  public void drive(double lf_degrees, double lf_speed,
   double lr_degrees, double lr_speed,
   double rf_degrees, double rf_speed,
   double rr_degrees, double rr_speed) {
    this.lf_degrees = lf_degrees; this.lf_speed = lf_speed;
    this.lr_degrees = lr_degrees; this.lr_speed = lr_speed;
    this.rf_degrees = rf_degrees; this.rf_speed = rf_speed;
    this.rr_degrees = rr_degrees; this.rr_speed = rr_speed;
  }

  public void stop() {
    lf_degrees = 0; lf_speed = 0;
    lr_degrees = 0; lr_speed = 0;
    rf_degrees = 0; rf_speed = 0;
    rr_degrees = 0; rr_speed = 0;
  }
}
