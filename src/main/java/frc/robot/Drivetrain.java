package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
	// for limelight
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;

public class Drivetrain {
	private MecanumDrive m_robotDrive;
	private ADIS16470_IMU gyro = new ADIS16470_IMU();
	private double currentAngleTarget;

	private double xTarget;
	private double yTarget;

	Drivetrain(int kFrontLeftChannelF,int kRearLeftChannelF,int kFrontRightChannelF,int kRearRightChannelF,
				int kFrontLeftChannelR, int kRearLeftChannelR, int kFrontRightChannelR, int kRearRightChannelR) 
	{
		Spark frontLeftF = new Spark(kFrontLeftChannelF);
		Spark rearLeftF = new Spark(kRearLeftChannelF);
		Spark frontRightF = new Spark(kFrontRightChannelF);
		Spark rearRightF = new Spark(kRearRightChannelF);
		
		Spark frontLeftR = new Spark(kFrontLeftChannelR);
		Spark rearLeftR = new Spark(kRearLeftChannelR);
		Spark frontRightR = new Spark(kFrontRightChannelR);
		Spark rearRightR = new Spark(kRearRightChannelR);
		
		// Invert the right side motors.
		frontRightF.setInverted(true);
		frontRightR.setInverted(true);
		rearRightF.setInverted(true); 
		rearRightR.setInverted(true);

		MotorControllerGroup frontLeftGroup = new MotorControllerGroup(frontLeftF, frontLeftR);
		MotorControllerGroup rearLeftGroup = new MotorControllerGroup(rearLeftF, rearLeftR);
		MotorControllerGroup frontRightGroup = new MotorControllerGroup(frontRightF, frontRightR);
		MotorControllerGroup rearRightGroup = new MotorControllerGroup(rearRightF, rearRightR);

		m_robotDrive = new MecanumDrive(frontLeftGroup, rearLeftGroup, frontRightGroup, rearRightGroup);

		currentAngleTarget = gyro.getAngle();
	}

	void driveRobot (double fbAxis, double sAxis, double rotAxis) {
		driveRobot (fbAxis, sAxis, rotAxis, false);
	}

	void driveRobot (double fbAxis, double sAxis, double rotAxis, boolean fieldOriented) {
		if (Math.abs (rotAxis) > 0.2) {
			currentAngleTarget = gyro.getAngle();
		} else {
			// use gyro to stablize 
			double p = -0.033, i = 0, d = 0;
			PIDController gyroPID;
			gyroPID = new PIDController(p, i, d);
			rotAxis = gyroPID.calculate(gyro.getAngle(), currentAngleTarget);
			gyroPID.close();
			Math.min (0.7, Math.max (-0.7, rotAxis));
		}

		double speedLimit = 0.5;
		fbAxis *= speedLimit;
		rotAxis *= speedLimit;
		sAxis *= speedLimit;

		Rotation2d curAngle = Rotation2d.fromDegrees(gyro.getAngle());	// for field oriented driving
		if (fieldOriented) {
			m_robotDrive.driveCartesian(fbAxis, sAxis, rotAxis, curAngle);
		} else {
			m_robotDrive.driveCartesian(fbAxis, sAxis, rotAxis);
		}
	}
	void resetGyroTarget () {
		currentAngleTarget = gyro.getAngle();
	}
	/* camera offset
	 * front 21.5
	 * back 4.5
	 * height 36.5
	 */
	void driveToPosition (double X, double Y) {	//need to add tolerances
		if (LimelightHelpers.getBotPose("")[0] == 0 || LimelightHelpers.getBotPose("")[1] == 0) {
			LimelightHelpers.setLEDMode_ForceOn("");
			System.out.print("limelight zero:" + LimelightHelpers.getJSONDump(""));
		} else {
			double xDiff = X - (LimelightHelpers.getBotPose("")[0] * 39.37);
			double yDiff = Y - (LimelightHelpers.getBotPose("")[1] * 39.37);
			//double zDiff = Z - currentPos.getZ();
	
			double xAngle = Math.atan(xDiff/yDiff);
			double yAngle = Math.atan(yDiff/xDiff);
	
			xTarget = (xDiff/Math.max(Math.abs(xDiff),Math.abs(yDiff)))/2.0;
					// the -1 here depends on where the robot is facing?
			yTarget = -1 * (yDiff/Math.max(Math.abs(xDiff),Math.abs(yDiff)));
	
			LimelightHelpers.setLEDMode_ForceOff("");
			SmartDashboard.putNumber("auto_XTarget", xTarget);
			SmartDashboard.putNumber("auto_YTarget", yTarget);
			SmartDashboard.putNumber("auto_XDiff", xDiff);
			SmartDashboard.putNumber("auto_YDiff", yDiff);
			SmartDashboard.putNumber("auto_X", X);
			SmartDashboard.putNumber("auto_Y", Y);
			SmartDashboard.putNumber("auto_XCur", (LimelightHelpers.getBotPose("")[0] * 39.37));
			SmartDashboard.putNumber("auto_YCur", (LimelightHelpers.getBotPose("")[1] * 39.37));
		}
		driveRobot(xTarget, yTarget, 0.0, true);
	}

	public void logPosition() {
		System.out.print("limelight:" + LimelightHelpers.getBotPose("")[0] + ", " + LimelightHelpers.getBotPose("")[1] + ", " + LimelightHelpers.getBotPose("")[5]);
		System.out.print("limelight in:" + (LimelightHelpers.getBotPose("")[0] * 39.37) + ", " + (LimelightHelpers.getBotPose("")[1] * 39.37) + ", " + LimelightHelpers.getBotPose("")[5]);
	}
}
