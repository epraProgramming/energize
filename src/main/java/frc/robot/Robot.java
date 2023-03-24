// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.Ultrasonic;

//import frc.robot.LimelightHelpers.LimelightResults;
//import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

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

  private Drivetrain theDrivetrain;
    /* end drive train */

    /* shooter */
  private Shooter theShooter;
    /* end shooter */

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

  @Override
  public void robotInit() {

      /* init drivetrain */
    theDrivetrain = new Drivetrain(
              kFrontLeftChannelF, kRearLeftChannelF, kFrontRightChannelF, kRearRightChannelF,
              kFrontLeftChannelR, kRearLeftChannelR, kFrontRightChannelR, kRearRightChannelR
            );
      /* end drivetrain init */

      /* init shooter */
    theShooter = new Shooter(1, 2, 3, 0, 1); 
      /* end shooter init */
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

    theDrivetrain.driveRobot(leftY, leftX, rightX);
      /* end drive controls */
    
      /* shooting controls */
    if (opStick.getAButton()) { // high shelf 
      theShooter.shoot(2);
    }
    if (opStick.getBButton()) { // high on low battery 
      theShooter.shoot(3);
    }
    if (opStick.getYButton()) { // mid shelf 
      theShooter.shoot(1);
    }
    if (opStick.getXButton()) { // full speed lob shot
      theShooter.shoot(0);
    }

    if (opStick.getRightBumper()) { // intake full speed
      theShooter.intake();
    }
    if (opStick.getLeftBumper()) { // intake full speed backwards
      theShooter.intakeEject();
    }
      /* end of shooter control */

      /* flag control */ 
    flag.set(opStick.getRightY());
      /* end of flag control */
  }
}

