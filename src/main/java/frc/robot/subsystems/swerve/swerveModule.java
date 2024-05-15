package frc.robot.subsystems.swerve;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.PID;
import frc.robot.utils.tools;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class swerveModule {
	public final TalonSRX turningMotor;
	public final CANSparkMax driveMotor;

	public String name = "";

	public PID pid;

	private double turnValue;
	private double error;

	/**********functions**********/

	//init
	public swerveModule(int turningMotorID, int driveMotorID){
		turningMotor = new TalonSRX(turningMotorID);
		driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushed);
	}

	//set motor power
	public void setpoint(double speed, double angle){
		error = angle - turnValue;//SP - PV 

		if(error > 180) error -= 360;
		else if(error < -180) error += 360;

		if(-1 < error && error < 1){
			pid.resetIntergral();
			error = 0;
		}

		final double turnPower = 
			tools.bounding(pid.calculate(tools.bounding(error / 45.0, -1, 1)), -1, 1);

		final double drivePower = 
			speed * tools.bounding(Math.abs(error / -90.0 + 1.5) * 0.5, 0, 1);

		turningMotor.set(TalonSRXControlMode.PercentOutput, -turnPower);
		driveMotor.set(drivePower);
	}

	public void stopMoving(){
		pid.resetIntergral();

		turningMotor.set(TalonSRXControlMode.PercentOutput, 0);
		driveMotor.set(0);
	}

	public double getTurnError(){
		return error;
	}

	//get encoder value
	public void getEncValue(){
		turnValue = -(double)((int)turningMotor.getSelectedSensorPosition() & 0x03ff) * 0.3515625;

		if(turnValue < 0) turnValue += 360;
		if(turnValue >= 360) turnValue -= 360;

		SmartDashboard.putNumber(name, turnValue);
	}

}
