package org.usfirst.frc.team1592.robot.swerve;

import org.usfirst.frc.team1592.robot.swerve.CANTalonPID;
import org.usfirst.frc.team1592.robot.swerve.Vector2D;
import org.usfirst.frc.team1592.robot.Constants;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

public class SwervePod {
	private final CANTalon driveMotor;
	private final CANTalon rotateMotor;

	//Radius of the drive wheel
	private final double radiusWheel;
	//Reduction from encoder to the wheel axle
	private final double enc2whlRatio;
	//Encoder mounting offset [revs]
	private final double encBias_revs;
	//Position of the pod on the robot (likely from the center)
	private final Vector2D position;
	//revolutions per minute to radians per second
	private static final double kRPM2RPS = Math.PI / 30.0;
	
	/**
	 * Constructor
	 * @param drvTalon<int> drive motor controller channel ID
	 * @param rotTalon<int> rotation motor controller channel ID
	 * @param location<Vector2D> Position of the pod on the robot (likely from the center)
	 * @param encBias<double> encoder reading in revs when the wheel is at null position
	 * @param rotatePID<PIDConfig> container of PID constants for pod rotation
	 * Assumes rotation encoder is mounted directly to the wheel (1:1 ratio)
	 */
	public SwervePod(int drvTalonID, int rotTalonID, Vector2D location, double encBias, double wheelRad, CANTalonPID rotatePID, double enc2whlRatio) {
		
		//Instance of the drive motor controller
		this.driveMotor = new CANTalon(drvTalonID);
		//Instance of the rotation motor controller
		this.rotateMotor = new CANTalon(rotTalonID);
		//Radius of the drive wheel
		this.radiusWheel = wheelRad;
		//Reduction from encoder to the wheel axle
		this.enc2whlRatio = enc2whlRatio;
		//Encoder mounting offset [revs]
		this.encBias_revs = encBias;
		this.position = location;
		
		//TODO: move the config out of the pod (to RobotMap) to be generic? this would remove the TalonPID settings
		driveMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	//configure rotation talon
    	rotateMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	rotateMotor.changeControlMode(TalonControlMode.Position);
    	rotateMotor.setPID(rotatePID.kP, rotatePID.kI, rotatePID.kD);
    	rotateMotor.setF(rotatePID.kF);
    	rotateMotor.setIZone(rotatePID.kIz);
    	//Set the pod to it's current position in revs
    	rotateMotor.setPosition(getAbsAngle()/360d);
	}
	
	/**
	 * Set pod wheel command in polar coordinates. Wheel computes the shortest path to take
	 * @param angle: commanded pod angle [deg]
	 * @param driveCmd: commanded wheel speed
	 */
	public void set(double angleDeg, double driveCmd) {
		double theta = getWheelAngle_Deg();
		double deltaTheta = angleDeg - theta;
		//Unwrap deltaTheta to (-360,360)
		deltaTheta = deltaTheta % 360;
		//Unwrap deltaTheta to (0,360)
		if (deltaTheta < 0) {deltaTheta +=360d;}
		//Put deltaTheta on [-180,180) to allow 1-sided bounds checking
		deltaTheta -= 180d;
		//Shifting by 180 defaults to inverting
		Boolean shouldInvert = true;
		//If in quadrant 2
		if(deltaTheta > 90) {
			deltaTheta -= 180;
			shouldInvert = false;
			//If in quadrant 3
		} else if(deltaTheta <-90) {
			deltaTheta += 180;
			shouldInvert = false;
		}
		//Send the command to the drive motor
		//Using delta-angle prevents wheel from unwrapping
		rotateMotor.set((deltaTheta + theta)/360d);
		//TODO: should we subtract off the pod rotation velocity since they're coupled?
		driveMotor.set(shouldInvert?-driveCmd:driveCmd);
	}
	
	/**
	 * Changes the talon control mode for the pod drive motor
	 *
	 * @param mode  the new mode
	 */
	public void changeDriveControlMode(TalonControlMode mode) {
		driveMotor.changeControlMode(mode);
	}
	
	/**
	 * Sets the drive motor profiles for the pod drive motor.
	 */
	public void establishDriveProfiles() {
		driveMotor.setProfile(1);
		driveMotor.setPID(0.0,0.0,0.0);
		driveMotor.setF(Constants.LFSwerve.Kf);
		driveMotor.setProfile(0);
    	Constants.driveWheelPID.applyConstants(driveMotor);
	}
	
	/**
	 * Sets the drive profile to the specified slot.
	 *
	 * @param slot  PID parameter slot on the Talon
	 */
	public void setDriveProfile(int slot) {
		driveMotor.setProfile(slot);
	}
	
	/**
	 * Get raw pod angle off the absolute encoder
	 * @return angle on range [0,360) deg
	 */
	public double getAbsAngle() {
		//Absolute position does wrap, so use modulus to put on range (-360,360)
		double ang = ((rotateMotor.getPulseWidthPosition()/4096d)-encBias_revs)*360d%360;
		//Put on range [0,360)
		if(ang < 0)
			ang += 360; 
		return ang;
	}
	
	/**
	 * Get pod location on the robot
	 * @return <Vector> position
	 */
	public Vector2D getPosition() {
		return position;
	}
	
	/**
	 * Get wheel velocity
	 * @return velocity of wheel in RPM
	 */
	public double getWheelSpeed_RPM() {
		//get encoder speed and convert from encoder RPM to wheel RPM
		return driveMotor.getSpeed() * enc2whlRatio;
	}
	
	/**
	 * Get pod angle
	 * @return angle in revolutions
	 */
	public double getWheelAngle_Revs() {
		return rotateMotor.getPosition();
	}
	
	/**
	 * Get pod angle
	 * @return angle in degrees
	 */
	public double getWheelAngle_Deg() {
		return rotateMotor.getPosition()*360d;
	}
	
	/**
	 * Set rotate motor PID constants
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	public void setRotatePID(double kP, double kI, double kD) {
		rotateMotor.setPID(kP, kI, kD);
	}
	
	/**
	 * Get pod velocity in robot coordinate frame
	 * @return velocity <Vector> in feet per second
	 */
	public Vector2D getVelocity_FPS() {
		//get encoder speed and convert from encoder RPM to wheel FPS
		double vel = getWheelSpeed_RPM() * kRPM2RPS * radiusWheel;//[FPS]
		//get pod position and convert from revs to radians
		double angle = getWheelAngle_Revs() * 2.0 * Math.PI; 	//[rad]
		//rotate wheel velocity into the body frame
		return new Vector2D(vel*Math.cos(angle),vel*Math.sin(angle));
	}
}
