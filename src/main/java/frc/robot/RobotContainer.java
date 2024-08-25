package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/*** subsystems ***/

import frc.robot.subsystems.Swerve;

/*** commands ***/

import frc.robot.commands.JoystickSwerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();

  private final Joystick js1 = new Joystick(Constants.OperatorConstants.Player1Port);

  public RobotContainer() {
    player1CommandList();
  }

  private void player1CommandList() {
    // swerve
    swerve.setDefaultCommand(new JoystickSwerve(
      swerve, 
      ()->js1.getX(),
      ()->js1.getY(), 
      ()->js1.getRawAxis(4))
    );
  }

    public Command getAutonomousCommand() {
    return new PrintCommand("None auto command running!");
  }
}
