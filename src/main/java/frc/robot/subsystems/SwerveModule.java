package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.PID;
import frc.robot.utils.tools;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveModule {
	public final TalonSRX turningMotor;
	public final CANSparkMax driveMotor;

	public final String name;

	private final PID pid;

	public double turnValue;

	/**********functions**********/

	public SwerveModule(int turningMotorID, int driveMotorID, String name){
		turningMotor = new TalonSRX(turningMotorID);
		driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushed);
		this.name = name;

		pid = new PID(8, 1e-3, 0.3);
	}

	//set motor power
	public void setpoint(double speed, double angle){
		double error = angle - turnValue;

		error = error > 180 ? error - 360 : error;
		error = error < -180 ? error + 360 : error;

		if(-5 < error && error < 5){
			pid.resetIntergral();
			error = 0;
		}

		error /= 180.0;

		double turnPower = pid.calculate(error);

		tools.bounding(turnPower, -1, 1);

		turningMotor.set(TalonSRXControlMode.PercentOutput, -turnPower);
		driveMotor.set(0);
	}

	public void stop() {
		turningMotor.set(TalonSRXControlMode.PercentOutput, 0);
		driveMotor.set(0);
	}

	//get encoder value
	public void getEncValue(){
		turnValue = -((int)turningMotor.getSelectedSensorPosition() & 0x03ff) * 0.3515625;

		turnValue = turnValue < 0 ? turnValue + 360 : turnValue;
		turnValue = turnValue >= 360 ? turnValue - 360 : turnValue;

		SmartDashboard.putNumber(name, turnValue);
	}
}
