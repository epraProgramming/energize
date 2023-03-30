// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* arm and claw */
    /*private CANSparkMax clawFront = new CANSparkMax (1, MotorType.kBrushless); 
    private CANSparkMax clawBack = new CANSparkMax (2, MotorType.kBrushless);
    private CANSparkMax arm = new CANSparkMax (3, MotorType.kBrushless);*/
/* end of arm and claw */
/* claw controls */
      /*clawFront.set(opStick.getLeftX()); 
      clawBack.set(-1 * opStick.getLeftX());
      arm.set(opStick.getRightY());*/
/* end of claw controls */

/** Add your docs here. */
public class Claw {
	private CANSparkMax clawFront; 
	private CANSparkMax clawBack;
	private CANSparkMax arm;
	
	Claw(int clawFrontId, int clawBackId, int armId) {
		clawFront  = new CANSparkMax (clawFrontId, MotorType.kBrushless);
		clawBack  = new CANSparkMax (clawBackId, MotorType.kBrushless);
		arm  = new CANSparkMax (armId, MotorType.kBrushless);
	}

	public void pickup () {
		clawFront.set(1);
		clawBack.set(-1);
	}

	public void release () {
		clawFront.set(-1);
		clawBack.set(1);
	}

	public void armToPosition (int targetHeight) {
		double target = 0;
		switch (targetHeight) {
			case 0:		// resting place
				target = 0;
				break;
			case 1:		// low node
				target = 5;
				break;
			case 2:		// high node
				target = 7;
				break;
			case 3: 	// human player station
				target = 8;
				break;
		}
		
		//arm.setTargetPosition(target);
	}

}
