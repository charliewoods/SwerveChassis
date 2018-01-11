package org.usfirst.frc.team1592.robot;

import org.usfirst.frc.team1592.robot.swerve.SwerveDrive;
import org.usfirst.frc.team1592.robot.swerve.SwervePod;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public static AHRS navX=new AHRS(SPI.Port.kMXP);
    public static SwervePod leftFrontPod = new SwervePod(Constants.LFSwerve.DRIVE_CHAN, Constants.LFSwerve.ROTATE_CHAN, Constants.LFSwerve.LOC, Constants.LFSwerve.OFFSET, Constants.WHEEL_RAD, Constants.rotatePodPID, Constants.ENC2AXLE_RATIO);
    public static SwervePod rightFrontPod = new SwervePod(Constants.RFSwerve.DRIVE_CHAN, Constants.RFSwerve.ROTATE_CHAN, Constants.RFSwerve.LOC, Constants.RFSwerve.OFFSET, Constants.WHEEL_RAD, Constants.rotatePodPID, Constants.ENC2AXLE_RATIO);
    public static SwervePod leftBackPod = new SwervePod(Constants.LBSwerve.DRIVE_CHAN, Constants.LBSwerve.ROTATE_CHAN, Constants.LBSwerve.LOC, Constants.LBSwerve.OFFSET, Constants.WHEEL_RAD, Constants.rotatePodPID, Constants.ENC2AXLE_RATIO);
    public static SwervePod rightBackPod = new SwervePod(Constants.RBSwerve.DRIVE_CHAN, Constants.RBSwerve.ROTATE_CHAN, Constants.RBSwerve.LOC, Constants.RBSwerve.OFFSET, Constants.WHEEL_RAD, Constants.rotatePodPID, Constants.ENC2AXLE_RATIO);
    public static SwerveDrive swerveDrive = new SwerveDrive(leftFrontPod, rightFrontPod, leftBackPod, rightBackPod);
}
