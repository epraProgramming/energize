// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Ultrasonic;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
    /* drive train */
  private static final int kFrontLeftChannelF = 0;
  private static final int kRearLeftChannelF = 2;
  private static final int kFrontRightChannelF = 4;
  private static final int kRearRightChannelF = 6;

  private static final int kFrontLeftChannelR = 1;
  private static final int kRearLeftChannelR = 3;
  private static final int kFrontRightChannelR = 5;
  private static final int kRearRightChannelR = 7;

/* For the second test robot
  private static final int kFrontLeftChannel = 4;
  private static final int kRearLeftChannel = 0;
  private static final int kFrontRightChannel = 6;
  private static final int kRearRightChannel = 2;

  private static final int kFrontLeftChannelMini = 5;
  private static final int kRearLeftChannelMini = 1;
  private static final int kFrontRightChannelMini = 7;
  private static final int kRearRightChannelMini = 3;
 */

  private MecanumDrive m_robotDrive;
    /* end drive train */

    /* arm and claw */
    private CANSparkMax clawFront = new CANSparkMax (1, MotorType.kBrushless); 
    private CANSparkMax clawBack = new CANSparkMax (2, MotorType.kBrushless);
    private CANSparkMax arm = new CANSparkMax (3, MotorType.kBrushless);
      /* end of arm and claw */
  
      /* shooter */
    private CANSparkMax advancer = new CANSparkMax (4, MotorType.kBrushless);
    private CANSparkMax shooterFront = new CANSparkMax (5, MotorType.kBrushless); 
    private CANSparkMax shooterBack = new CANSparkMax (6, MotorType.kBrushless);
      /* end of shooter */

      /* flag */
    private CANSparkMax flag = new CANSparkMax (7, MotorType.kBrushed);
      /* end of flag */
  
      /* controllers */
    private final XboxController driveStick = new XboxController(0);
    private final XboxController opStick = new XboxController(1);
      /* these will be from the driver sticks but can be overwritten by other processes */
    private double leftY = 0;
    private double leftX = 0;
    private double rightX = 0;
      /* end controllers */
/* drive train is being defined here for single motor testing */
Spark frontLeftF = new Spark(kFrontLeftChannelF);
Spark rearLeftF = new Spark(kRearLeftChannelF);
Spark frontRightF = new Spark(kFrontRightChannelF);
Spark rearRightF = new Spark(kRearRightChannelF);

Spark frontLeftR = new Spark(kFrontLeftChannelR);
Spark rearLeftR = new Spark(kRearLeftChannelR);
Spark frontRightR = new Spark(kFrontRightChannelR);
Spark rearRightR = new Spark(kRearRightChannelR);

  @Override
  public void robotInit() {
      /* init drivetrain * /
    Spark frontLeftF = new Spark(kFrontLeftChannelF);
		Spark rearLeftF = new Spark(kRearLeftChannelF);
		Spark frontRightF = new Spark(kFrontRightChannelF);
		Spark rearRightF = new Spark(kRearRightChannelF);

		Spark frontLeftR = new Spark(kFrontLeftChannelR);
		Spark rearLeftR = new Spark(kRearLeftChannelR);
		Spark frontRightR = new Spark(kFrontRightChannelR);
		Spark rearRightR = new Spark(kRearRightChannelR);
*/
		// Invert the right side motors.
		frontRightF.setInverted(true);
		frontRightR.setInverted(true);
		rearRightR.setInverted(true);
		rearRightR.setInverted(true);

		MotorControllerGroup frontLeftGroup = new MotorControllerGroup(frontLeftF, frontLeftR);
		MotorControllerGroup rearLeftGroup = new MotorControllerGroup(rearLeftF, rearLeftR);
		MotorControllerGroup frontRightGroup = new MotorControllerGroup(frontRightF, frontRightR);
		MotorControllerGroup rearRightGroup = new MotorControllerGroup(rearRightF, rearRightR);

		m_robotDrive = new MecanumDrive(frontLeftGroup, rearLeftGroup, frontRightGroup, rearRightGroup);
      /* end drivetrain init */

      /* controller init */
    }

  private double deadband (double inValue, double minValue) {
    if (Math.abs(inValue) >= minValue) {
      return inValue;
    } else {
      return 0;
    }
  }
  
  @Override
  public void teleopPeriodic() {
      /* drive controls */
    leftY = -1 * deadband(driveStick.getLeftY(), 0.5);
    leftX = 1 * deadband(driveStick.getLeftX(), 0.5);
    rightX = -1 * deadband(driveStick.getRightX(), 0.5); 

    if (driveStick.getRightBumper()) {
      leftY = leftY * 0.35;
      leftX = leftX * 0.35;
      rightX = rightX * 0.35;
    }
//    m_robotDrive.driveCartesian(leftY, leftX, rightX);
if (driveStick.getAButton()) {
  frontLeftF.set(driveStick.getLeftY());
  frontRightF.set(driveStick.getRightY());
}
if (driveStick.getBButton()) {
  frontLeftR.set(driveStick.getLeftY());
  frontRightR.set(driveStick.getRightY());
}
if (driveStick.getXButton()) {
  rearLeftF.set(driveStick.getLeftY());
  rearRightF.set(driveStick.getRightY());
}
if (driveStick.getYButton()) {
  rearLeftR.set(driveStick.getLeftY());
  rearRightR.set(driveStick.getRightY());
}

/* for field oriented driving use these
    curAngle = Rotation2d.fromDegrees(gyro.getAngle());	// for field oriented driving
		m_robotDrive.driveCartesian(leftY, leftX, rightX, curAngle);
    */
      /* end drive controls */

      /* claw controls */
      clawFront.set(opStick.getLeftX()); 
      clawBack.set(-1 * opStick.getLeftX());
      arm.set(opStick.getRightY());
      /* end of claw controls */

      /* shooter control */
    shooterBack.set (opStick.getLeftTriggerAxis() - opStick.getRightTriggerAxis());
    shooterFront.set (-1 * (opStick.getLeftTriggerAxis() - opStick.getRightTriggerAxis()));
    double aSpeed = 0;
    if (opStick.getAButton()) {
      aSpeed = 0.5;
    }
    if (opStick.getBButton()) {
      aSpeed = -0.5;
    }
    if (opStick.getYButton()) {
      aSpeed = 1.0;
    }
    if (opStick.getXButton()) {
      aSpeed = -1.0;
    }
    advancer.set (aSpeed);
      /* end of shooter control */

      /* flag control */
    if (opStick.getLeftBumper()) {
      flag.set(0.5);
    } else if (opStick.getRightBumper()) {
      flag.set(-0.5);
    } else {
      flag.set(0);
    }
      /* end of flag control */
    }
}
