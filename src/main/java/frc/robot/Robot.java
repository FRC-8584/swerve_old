package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.swerve;

import frc.robot.utils.tools;
import frc.robot.utils.sensors;

import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
  private static final Joystick player1 = new Joystick(0);

  @Override
  public void robotInit() {
    sensors.initGyro();
    swerve.init();
  }

  @Override
  public void robotPeriodic() {
    swerve.getEncValue();
    swerve.getRobotHeading();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    swerve.move(0, 0, 0);
  }

  @Override
  public void teleopPeriodic() {
    System.out.println(sensors.gyro.getVector()[0]);
    singlePlayer();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /*********************** Functions ***********************/
  
  public void singlePlayer() {
    //Swerve
    double x, y, turnRobot;

    // ~ Player1 ~

    x               =  player1.getRawAxis(0);
    y               = -player1.getRawAxis(1);
    turnRobot       =  player1.getRawAxis(4);

    /***** Check deadband *****/

    if(-0.05 < x && x < 0.05) x = 0;
    if(-0.05 < y && y < 0.05) y = 0;
    if(-0.05 < turnRobot && turnRobot < 0.05) turnRobot = 0;

    /******* Set point *******/

    swerve.move(x, y, turnRobot);

    SmartDashboard.putNumber("input", tools.toDegrees(x, y));
  }
}

/*









 
`                   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
`                   * O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O *
`                   * O                                                                     O *
`                   * O                                  _,.=*.                             O *
`                   * O                          __,.=*"'      ".                           O *
`                   * O                         \      _  .=**"'`                           O *
`                   * O                          "===*"H  H                                 O *
`                   * O                                H  H                                 O *
`                   * O                               _H  H___...===***"""***q.             O *
`                   * O           __......====****"""''    ___...===****==.___.b            O *
`                   * O           \      ___...===***""H  H                                 O *
`                   * O            `**""'        qxp   H  H   qxp                           O *
`                   * O                          l l   H  H   l l                           O *
`                   * O                          l l   H  H   l l                           O *
`                   * O                    ___...a l   H  H   l `***""'````*q.              O *
`                   * O             .q"""' ___..., l   H  H   l ..=====..__   \             O *
`                   * O              `a*"''      l l   H  H   l l          ``"'             O *
`                   * O                          l l   H  H   l l             .             O *
`                   * O                          l l   H  H   l l            yH             O *
`                   * O                         _i b   H  H   l l           y H             O *
`                   * O                     _-*' _-*   H  H   b  b         y  j             O *
`                   * O                 _-*' _-*'      H  H    b  '*=====*'  y              O *
`                   * O            .d""' _-*'          H  H     b-_________.d               O *
`                   * O             `a*"'              H  H                                 O *
`                   * O                                q  p                                 O *
`                   * O                                'qp'                                 O *
`                   * O                                                                     O *
`                   * O                                                                     O *
`                   * O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O *
`                   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 









*/
