package frc;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;


public class drivetrain {
	private MecanumDrive m_robotDrive;

	drivetrain(int kFrontLeftChannelF,int kRearLeftChannelF,int kFrontRightChannelF,int kRearRightChannelF,
	int kFrontLeftChannelR, int kRearLeftChannelR, int kFrontRightChannelR, int kRearRightChannelR) {
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
		rearRightR.setInverted(true);
		rearRightR.setInverted(true);

		MotorControllerGroup frontLeftGroup = new MotorControllerGroup(frontLeftF, frontLeftR);
		MotorControllerGroup rearLeftGroup = new MotorControllerGroup(rearLeftF, rearLeftR);
		MotorControllerGroup frontRightGroup = new MotorControllerGroup(frontRightF, frontRightR);
		MotorControllerGroup rearRightGroup = new MotorControllerGroup(rearRightF, rearRightR);

		m_robotDrive = new MecanumDrive(frontLeftGroup, rearLeftGroup, frontRightGroup, rearRightGroup);
	}

	void driveRobot (double fbAxis, double lrAxis, double sAxis) {
		m_robotDrive.driveCartesian(fbAxis, lrAxis, sAxis);
	}

	/* camera offset
	 * front 21.5
	 * back 4.5
	 * height 36.5
	 */
	void driveToPosition (double X, double Y, double Z, Pose3d currentPos) {	//need to add tolerances
		double xDiff = X - currentPos.getX();
		double yDiff = Y - currentPos.getY();
		double zDiff = Z - currentPos.getZ();

		double xAngle = Math.atan(xDiff/yDiff);
		double yAngle = Math.atan(yDiff/xDiff);

	}
}
