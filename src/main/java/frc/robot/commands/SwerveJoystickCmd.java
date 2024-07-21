package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.driveConstants;
import frc.robot.utils.tools;

import java.util.function.Supplier;

public class SwerveJoystickCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private Supplier<Double> x, y, turn;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> x, Supplier<Double> y, Supplier<Double> turn) {
    this.swerveSubsystem = swerveSubsystem;
    this.x = x;
    this.y = y;
    this.turn = turn;

    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.stop();
  }

  @Override
  public void execute() {
    double x = this.x.get();
    double y = this.y.get();
    double turn = this.turn.get();

    if(-0.05 < x && x < 0.05) x = 0;
    if(-0.05 < y && y < 0.05) y = 0;
    if(-0.05 < turn && turn < 0.05) turn = 0;

    // System.out.println(x + " " + y + " " + turn);

    //convert the vector of driver's heading to the vector of robot's heading

    // convertHeading();

    x *= driveConstants.kx; y *= driveConstants.ky; turn *= driveConstants.kturn;

    //calculate vector (x, y)

    final double lf_x = x + turn*(driveConstants.length / driveConstants.r), lf_y = y + turn*(driveConstants.width / driveConstants.r);
    final double lr_x = x - turn*(driveConstants.length / driveConstants.r), lr_y = y + turn*(driveConstants.width / driveConstants.r);
    final double rf_x = x + turn*(driveConstants.length / driveConstants.r), rf_y = y - turn*(driveConstants.width / driveConstants.r);
    final double rr_x = x - turn*(driveConstants.length / driveConstants.r), rr_y = y - turn*(driveConstants.width / driveConstants.r);

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

    double max = 1;
    max = lf_speed > max ? lf_speed : max;
    max = lr_speed > max ? lr_speed : max;
    max = rf_speed > max ? rf_speed : max;
    max = rr_speed > max ? rr_speed : max;

    lf_speed /= max;
    lr_speed /= max;
    rf_speed /= max;
    rr_speed /= max;

		//setpoint

    // System.out.println(lf_degrees + " " + lf_speed + " " + lr_degrees + " " + lr_speed + " " + rf_degrees + " " + rf_speed + " " + rr_degrees + " " + rr_speed);

    swerveSubsystem.drive(lf_degrees, lf_speed, lr_degrees, lr_speed, rf_degrees, rf_speed, rr_degrees, rr_speed);
    
    return;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
