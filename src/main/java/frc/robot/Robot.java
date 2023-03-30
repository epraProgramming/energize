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

  private Timer theTimer;

  private Drivetrain theDrivetrain;
    /* end drive train */

    /* shooter */
  private Shooter theShooter;
    /* end shooter */

    /* flag */
//  private CANSparkMax flag = new CANSparkMax (7, MotorType.kBrushed);
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
      /* init shooter */
    theShooter = new Shooter(6, 5, 4, 0, 1, 3, 7); 
      /* end shooter init */
    
      /* init drivetrain */
    theDrivetrain = new Drivetrain(
      kFrontLeftChannelF, kRearLeftChannelF, kFrontRightChannelF, kRearRightChannelF,
      kFrontLeftChannelR, kRearLeftChannelR, kFrontRightChannelR, kRearRightChannelR
    );
      /* end drivetrain init */
  }

  private double deadband (double inValue, double minValue) {
    if (Math.abs(inValue) >= minValue) {
      return inValue;
    } else {
      return 0;
    }
  }
  
  public void autonomousInit() {
  theTimer = new Timer(0);
  }

  public void autonomousPeriodic() {
    double autoStep = 1;
    /* Auto for ONE or more cubes */
    // step 1
    // Launch starting cube
    if (autoStep == 1) {
      theTimer.reset(1000);
      theShooter.intake();
      theShooter.shoot(0); // does a lob shot from starting position
      if (theTimer.timerElasped() == true) {
        autoStep = 2;
      }
    }
    /* Auto for TWO or more cubes */
    // step 2
    // Drop flag
    if (autoStep == 2) {
      theTimer.reset(1000);
      theShooter.dropFlag();
      if (theTimer.timerElasped() == true) {
        autoStep = 3;
      }
    }
    // step 3
    // Drive backwards
    
    // step 4
    // Intake cube 
    if (autoStep == 4) {
      theTimer.reset(1000);
      theShooter.intake();
      if (theTimer.timerElasped() == true) {
        autoStep = 5;
      }
    }
    // step 5
    // Drive foward 

    // step 6
    // Shoot cube and move sideways

    /* Auto for THREE or more cubes, otherwise jump to step 19 */
    // step 7
    // Drive backward

    // step 8
    // Intake cube

    // step 9
    // Drive foward

    // step 10
    // Shoot cube and move sideways

    /* Auto for FOUR or more cubes, otherwise jump to step 19 */
    // step 11
    // Drive backward

    // step 12
    // Intake cube

    // step 13
    // Drive foward

    // step 14
    // Shoot cube and move sideways

    /* Auto for all FIVE cubes, otherwise jump to step 19 */
    // step 15
    // Drive backward

    // step 16 
    // Intake cube

    // step 17
    // Drive forward

    // step 18 
    // Shoot cube
    
    /* Autos 2, 3, 4, and 5 use this step */
    // step 19
    // Raise flag
    if (autoStep == 19) {
      theTimer.reset(1000);
      theShooter.raiseFlag();
      if (theTimer.timerElasped() == true) {
        autoStep = 20;
      }
    }
  } // end of autonomousPeriodic

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

    if (driveStick.getLeftBumperPressed()) {
      theDrivetrain.logPosition();
    }

    if (driveStick.getBButtonPressed()) {
      theDrivetrain.resetGyroTarget();
    }
    if (driveStick.getAButton()){
      theDrivetrain.driveToPosition(47.36, -73.61);
    } else {
      theDrivetrain.driveRobot(leftY, leftX, rightX);
    }
      /* end drive controls */
    
      /* shooting controls */
    if (opStick.getAButton()) { // high shelf 
      theShooter.shoot(2);
      theShooter.intake();
    }
    if (opStick.getBButton()) { // high on low battery 
      theShooter.shoot(3);
      theShooter.intake();
    }
    if (opStick.getYButton()) { // mid shelf 
      theShooter.shoot(1);
      theShooter.intake();
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
    //flag.set(opStick.getRightY());
      /* end of flag control */
  } // end of teleopPeriodic
}

