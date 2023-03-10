// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter {
	
	private CANSparkMax advancer;
	private CANSparkMax shooterFront;
	private CANSparkMax shooterBack;
	private DigitalInput cubeDetector;

	Shooter(int shooterAdvancerId, int shooterFrontId, int shooterBackId, int cubeDetectorId) {
		advancer = new CANSparkMax(shooterAdvancerId, MotorType.kBrushless);
		shooterFront = new CANSparkMax(shooterFrontId, MotorType.kBrushless);
		shooterBack = new CANSparkMax(shooterBackId, MotorType.kBrushless);
		cubeDetector = new DigitalInput(cubeDetectorId);
	}

	public void intake() {
		advancer.set(1);
	}

	public void intakeEject() {
		advancer.set(-1);
	}

	//shooter code not yet done
	boolean activeShooter = false;
	int shooterStep = 0;
	boolean sawCube = false;

	public void shoot(int speedTarget) {
		double curShooterFront = shooterFront.get();
		double curShooterBack = shooterBack.get();
		activeShooter = true;
		double targetSpeed = 0;
		switch (speedTarget) {
			case 0: // full speed foward
			targetSpeed = 1;
			break;
			case 1: // full speed backward
			targetSpeed = -1;
			break;
			case 2: // half speed foward
			targetSpeed = 0.5;
			break;
			case 3: // half speed backward
			targetSpeed = -0.5;
			break;
		} 
		shooterFront.set(targetSpeed); 
		shooterBack.set(targetSpeed * -1); // shooterBack needs reverse speed value

		if (shooterStep == 0 && curShooterFront == targetSpeed && curShooterBack == targetSpeed) {
			shooterStep = 1;
		}

		if (shooterStep == 1) {
			advancer.set(targetSpeed);
		
			if (cubeDetector.get() == false) {
			sawCube = true;
			}
			if (cubeDetector.get() == true && sawCube == true) {
			activeShooter = false;
			sawCube = false;
			advancer.set(0);
			shooterFront.set(0);
			shooterBack.set(0);
			shooterStep = 0;
			targetSpeed = 0;
			return;
			}
		}
	}

	// grabbed code from early robot tesing code
	/* shooter control */
	/*public void ShooterControl(XboxController shooterControlStick) {
    shooterBack.set (shooterControlStick.getLeftTriggerAxis() - shooterControlStick.getRightTriggerAxis());
    shooterFront.set (-1 * (shooterControlStick.getLeftTriggerAxis() - shooterControlStick.getRightTriggerAxis()));
    double aSpeed = 0;
    if (shooterControlStick.getAButton()) {
      aSpeed = 0.5;
    }
    if (shooterControlStick.getBButton()) {
      aSpeed = -0.5;
    }
    if (shooterControlStick.getYButton()) {
      aSpeed = 1.0;
    }
    if (shooterControlStick.getXButton()) {
      aSpeed = -1.0;
    }
    advancer.set (aSpeed); 
	}*/ 
	/* end of shooter control */

} // end public class Shooter
