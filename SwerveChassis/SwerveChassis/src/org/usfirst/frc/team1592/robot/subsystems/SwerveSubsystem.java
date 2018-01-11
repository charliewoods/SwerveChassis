package org.usfirst.frc.team1592.robot.subsystems;

import org.usfirst.frc.team1592.robot.swerve.PIDConstants;
import org.usfirst.frc.team1592.robot.swerve.SwerveDrive;
import org.usfirst.frc.team1592.robot.subsystems.PIDSubsystem1592;
import org.usfirst.frc.team1592.robot.swerve.Vector2D;
import org.usfirst.frc.team1592.robot.Constants;
import org.usfirst.frc.team1592.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1592.robot.commands.DriveWithJoysticks_FPS;

import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;
import org.usfirst.frc.team1592.robot.subsystems.RateLimiterND;

public class SwerveSubsystem extends PIDSubsystem1592 {
	private final AHRS mImu;
	private final SwerveDrive mDrive;
	
	private double mRotationRateCmd;
	private double mYawAngleBias = 0.0; //bias on navX Angle
	private boolean mIsFieldOriented;
	
	//Rate Limiter 
	private boolean mIsRateLimiting = false;
	private static RateLimiterND lateralLimiter= new RateLimiterND(2,30.0*0.02);
	private static RateLimiterND rotateLimiter= new RateLimiterND(19.6 * 0.02);
	
	
	/**
	 * Constructor
	 * @param drive
	 * @param imu
	 * @param turnPID Heading controller parameters
	 */
	public SwerveSubsystem(SwerveDrive drive, AHRS imu, PIDConstants turnPID) {
		
    	super(turnPID.kP, turnPID.kI, turnPID.kD, turnPID.period);
    	getPIDController().setIZone(turnPID.iZone);
    	
    	mDrive = drive;
    	mImu = imu;
    	
    	mDrive.setMaxAllowableWheelSpeed(Constants.MAX_SPEED);
		mDrive.setPodRotateVelThreshold(Constants.DRIVE_THRESH);
		double kRPM2RPS = Math.PI / 30.0; //[RPS/RPM]
		mDrive.setInput2DriveCmdScale(1/ (Constants.WHEEL_RAD) / kRPM2RPS / Constants.ENC2AXLE_RATIO); //[RPM_enc / FPS_wheel]

		//=======PID Settings=========
		setSetpoint(getAngle_Deg());
		setOutputRange(-0.35*Constants.MAX_TURN_RATE, 0.35*Constants.MAX_TURN_RATE); //[Rad/s]
		setContinuous(true,0.0,360.0);
		setAbsoluteTolerance(1.5); 				//[deg]
		//    	getPIDController().setIZone(0.2*0.3*Constants.MAX_TURN_RATE); 				//[rad/s]
		disable(); //enable elsewhere if needed
		
		mDrive.changeDriveControlMode(TalonControlMode.Speed);
		mDrive.establishDriveProfiles();
		
    	//==========Swerve Configuration=============
		//NOTE: 0 is feed-forward, 1 is full PID
    	if (Constants.isVelDrive) setVelPIDFDrive(1);
    	else setPercentDrive();
    	
    	//=======Data to Log=========
//    	RobotMap.dataLogger.registerDoubleStream("GyroAngle", "deg", ()->getAngle(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("GyroRate", "deg/s", ()->getRate(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("RobotVelX_field", "ft/s", ()->getVelocityField_FPS().getX(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("RobotVelY_field", "ft/s", ()->getVelocityField_FPS().getY(),new DecimalFormat("#.##"));
//		
//    	RobotMap.dataLogger.registerDoubleStream("PodAngleLeftFront", "deg", ()->RobotMap.leftFrontPod.getWheelAngle_Revs()*360d,new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVelLeftFront", "RPM", ()->RobotMap.leftFrontPod.getWheelSpeed_RPM(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVoltLeftFront", "V", ()->RobotMap.leftFrontDrive.getOutputVoltage(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodAmpLeftFront", "A", ()->RobotMap.leftFrontDrive.getOutputCurrent(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerIntStream("LeftFrontDriveCLErr", "counts/100ms", ()->RobotMap.leftFrontDrive.getClosedLoopError());
//    	RobotMap.dataLogger.registerDoubleStream("LeftFrontDriveCLSetPnt", "RPM", ()->RobotMap.leftFrontDrive.getSetpoint(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("LeftFrontDriveCLSpeed", "RPM", ()->RobotMap.leftFrontDrive.getSpeed(),new DecimalFormat("#.##"));
//    	
//    	RobotMap.dataLogger.registerDoubleStream("PodAngleLeftBack", "deg", ()->RobotMap.leftBackPod.getWheelAngle_Revs()*360d,new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVelLeftBack", "RPM", ()->RobotMap.leftBackPod.getWheelSpeed_RPM(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVoltLeftBack", "V", ()->RobotMap.leftBackDrive.getOutputVoltage(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodAmpLeftBack", "A", ()->RobotMap.leftBackDrive.getOutputCurrent(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerIntStream("LeftBackDriveCLErr", "counts/100ms", ()->RobotMap.leftBackDrive.getClosedLoopError());
//    	RobotMap.dataLogger.registerDoubleStream("LeftBackDriveCLSetPnt", "RPM", ()->RobotMap.leftBackDrive.getSetpoint(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("LeftBackDriveCLSpeed", "RPM", ()->RobotMap.leftBackDrive.getSpeed(),new DecimalFormat("#.##"));
//    	
//    	RobotMap.dataLogger.registerDoubleStream("PodAngleRightFront", "deg", ()->RobotMap.rightFrontPod.getWheelAngle_Revs()*360d,new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVelRightFront", "RPM", ()->RobotMap.rightFrontPod.getWheelSpeed_RPM(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVoltRightFront", "V", ()->RobotMap.rightFrontDrive.getOutputVoltage(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodAmpRightFront", "A", ()->RobotMap.rightFrontDrive.getOutputCurrent(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerIntStream("RightFrontDriveCLErr", "counts/100ms", ()->RobotMap.rightFrontDrive.getClosedLoopError());
//    	RobotMap.dataLogger.registerDoubleStream("RightFrontDriveCLSetPnt", "RPM", ()->RobotMap.rightFrontDrive.getSetpoint(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("RightFrontDriveCLSpeed", "RPM", ()->RobotMap.rightFrontDrive.getSpeed(),new DecimalFormat("#.##"));
//    	
//    	RobotMap.dataLogger.registerDoubleStream("PodAngleRightBack", "deg", ()->RobotMap.rightBackPod.getWheelAngle_Revs()*360d,new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVelRightBack", "RPM", ()->RobotMap.rightBackPod.getWheelSpeed_RPM(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodVoltRightBack", "V", ()->RobotMap.rightBackDrive.getOutputVoltage(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("PodAmpRightBack", "A", ()->RobotMap.rightBackDrive.getOutputCurrent(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerIntStream("RightBackDriveCLErr", "counts/100ms", ()->RobotMap.rightBackDrive.getClosedLoopError());
//    	RobotMap.dataLogger.registerDoubleStream("RightBackDriveCLSetPnt", "RPM", ()->RobotMap.rightBackDrive.getSetpoint(),new DecimalFormat("#.##"));
//    	RobotMap.dataLogger.registerDoubleStream("RightBackDriveCLSpeed", "RPM", ()->RobotMap.rightBackDrive.getSpeed(),new DecimalFormat("#.##"));

	}
	
	/**
	 * Set robot into velocity control mode
	 * @param slot - PID parameter slot on the Talon
	 */
	public void setVelPIDFDrive(int slot) {
		
		//These are updated as needed
		mDrive.setDriveProfiles(slot);
		if (slot==1) {
			//Slot1 is feedforward
			//Disable ramping on joysticks
			DriveWithJoysticks_FPS.setRateLimiting(false);
			//Enable ramping in swerve drive
			mIsRateLimiting = true;
	    	setRamps(30*0.02, 19.6*0.02);
		} else {
			//Slot0 is full PID
			//Lower ramp is less responsive / more limiting
			//Enable ramping on joysticks
			DriveWithJoysticks_FPS.setRamps(4.0 * Constants.MAX_SPEED, 3.0 * Constants.MAX_TURN_RATE, 1.5 * Constants.MAX_SPEED);
			DriveWithJoysticks_FPS.setRateLimiting(true);
			//Disable ramping in SwerveDrive
			mIsRateLimiting = false;
		}
	}
	
	public void setPercentDrive() {
		//Enable ramps in SwerveDrive
		mIsRateLimiting = true;
    	setRamps(2.5*0.02, 2.5*0.02);
		//Disable ramping on joysticks
    	DriveWithJoysticks_FPS.setRateLimiting(false);
		//Set Drive scaling
		mDrive.setMaxAllowableWheelSpeed(1.0);
		mDrive.setInput2DriveCmdScale(1d);
		//Set Talon mode
		mDrive.changeDriveControlMode(TalonControlMode.PercentVbus);

    	
		//=======PID Settings=========
    	setSetpoint(getAngle_Deg());
    	setOutputRange(-0.5, 0.5);
    	setContinuous(true,0.0,360.0);
    	setAbsoluteTolerance(1.5);
    	getPIDController().setIZone(0.2);
    	disable(); //enable elsewhere if needed
	}
	
	/**
	 * Set rate limits on the chassis
	 * @param translate rate limit for lateral velocity commands [cmd_units/cycle]
	 * @param rotate rate limit for rotational velocity commands [cmd_units/cycle]
	 */
    public void setRamps(double translate, double rotate) {
    	lateralLimiter.setRateLimit(translate);
    	rotateLimiter.setRateLimit(rotate);
    }
    
    protected double returnPIDInput()
    {
    	return getAngle_Deg();
    }
    
    protected void usePIDOutput(double output)
    {
    	mRotationRateCmd = output;// normalize by battery voltage? getBatteryVoltage ;
    	/*
    	System.out.println("======================");
    	System.out.println("Set: " + getSetpoint());
    	System.out.println("Cur: " + getAngle());
    	System.out.println("Out: " + zRateCmd);
    	*/
    }
	
	protected void initDefaultCommand() {
		if (Constants.isVelDrive) setDefaultCommand(new DriveWithJoysticks_FPS());
		else setDefaultCommand(new DriveWithJoysticks());
	}
	
	/**
	 * Get robot velocity in the body frame in ft/sec
	 * @return velocity [ft/s]
	 */
	public Vector2D getVelocityBody_FPS() {
		return getSwerveDrive().getVelocityBody_FPS(Math.toRadians(getRate_DPS()));
	}
	
	/**
	 * Get robot velocity in the field frame in ft/sec
	 * @return velocity [ft/s]
	 */
	public Vector2D getVelocityField_FPS() {
		//Angle is positive for conversion from field to body; negative angle required from body to field
		return getVelocityBody_FPS().rotateByAngle(-getAngle_Deg());
	}
	
	/**
	 * Accepts joystick inputs to drive swerve
	 * @param fwd Forward velocity command
	 * @param left left velocity command
	 * @param z Rotation Rate
	 */
	public void driveSwerve(double fwd, double left, double z) {
		driveSwerve(fwd, left, z, new Vector2D(0, 0));
	}
	
	/**
	 * Drive in Polar coordinates
	 * @param ang: azimuth to drive towards
	 * @param mag: drive speed
	 * @param z: Rotation cmd (+ counter-clockwise)
	 */
	public void drivePolar(double ang, double mag, double z) {
		drivePolar(ang, mag, z, new Vector2D(0, 0));
	}
	
	/**
	 * Drive in Polar coordinates
	 * @param ang: azimuth to drive towards
	 * @param mag: drive speed
	 * @param z: Rotation cmd (+ counter-clockwise)
	 * @param pivotPoint: reference point from robot origin to pivot around
	 */
	public void drivePolar(double ang, double mag, double z, Vector2D pivotPoint) {
		double rad = Math.toRadians(ang);
		driveSwerve(mag * Math.cos(rad), mag * Math.sin(rad), z, pivotPoint);
	}
	
	/**
	 * Accepts joystick inputs to drive swerve around an arbitrary point
	 * Field oriented or not controlled by Chassis private param
	 * @param fwd Forward velocity command
	 * @param left left velocity command
	 * @param ccw Rotation Rate (+ counter-clockwise)
	 * @param pivotPoint in the chassis coordinate system
	 */
	public void driveSwerve(final double fwd,final double left,final double ccw, Vector2D pivotPoint) {
		Vector2D translationCmd = new Vector2D(fwd, left);
		double rateCmd = ccw;
		
		//If rate limiting, do it
		if (mIsRateLimiting) {
			double[] outLat = lateralLimiter.limitRate(fwd,left);
	    	double[] outRot = rotateLimiter.limitRate(ccw);
	    	translationCmd = new Vector2D(outLat[0], outLat[1]);
	    	rateCmd = outRot[0];
		}
		
		//if fieldOrented == true, Convert from field frame to body frame
    	if(isFieldOriented()) translationCmd = new Vector2D(fwd, left).rotateByAngle(getAngle_Deg());
    	
    	//Send drive commands to Swerve
		getSwerveDrive().drive(translationCmd.getX(), translationCmd.getY(),rateCmd, pivotPoint);
		
		//If rate limiting, we need to update the last state in case it was saturated by SwerveDrive
		if (mIsRateLimiting) {
			double[] lastCmd = this.getLastCommands();
			lateralLimiter.setLastValue(lastCmd[0],lastCmd[1]);
			rotateLimiter.setLastValue(lastCmd[2]);
		}
	}
	
	/**
	 * Get the last commands issued to SwerveDrive in the command frame (field or body)
	 * @return lastCommands<double[]> last commands {fwd,left,ccw}
	 */
	public double[] getLastCommands() {
		//Get the last commands in the body-fixed frame
		double[] lastCmd = mDrive.getLastCommands();
		//If commands were issues in the field frame, rotate the body commands into field
		if (isFieldOriented()) {
			Vector2D lastLatCmd = new Vector2D(lastCmd[0],lastCmd[1]).rotateByAngle(-getAngle_Deg());
			lastCmd[0] = lastLatCmd.getX();
			lastCmd[1] = lastLatCmd.getY();
		}
		return lastCmd;
	}
	
	/**
	 * Accepts inputs to drive swerve around an arbitrary point holding the heading set in the subsystem
	 * Field oriented or not controlled by Chassis private param
	 * @param fwd Forward velocity command
	 * @param left left velocity command
	 * @param pivotPoint in the chassis coordinate system
	 */
	public void drive2Heading(double fwd, double left, Vector2D pivotPoint) {
    	driveSwerve(fwd,left,getCommandedRate(),pivotPoint);
		//System.out.println("zrate cmd: " + zRateCmd);
	}
	
	/**
	 * Accepts inputs to drive swerve holding the heading set in the subsystem
	 * Field oriented or not controlled by Chassis private param
	 * @param fwd Forward velocity command
	 * @param left left velocity command
	 */
	public void drive2Heading(double fwd, double left) {
		this.drive2Heading(fwd, left, new Vector2D(0,0));
	}
	
	/**
	 * Tells the swerve to use field or robot oriented
	 * @param fo True if swerve should use field orientation
	 */
	public void setFieldOriented(boolean fo) {
		this.mIsFieldOriented = fo;
	}
	
	/**
	 * Returns whether the swerve is in field oriented mode
	 * @return True if swerve is field oriented
	 */
	public boolean isFieldOriented() {
		return mIsFieldOriented;
	}
	
	/**
	 * Set robot angle relative to the field
	 * @param angle [deg]
	 */
	public void setRobotAngle(double ang) {
		//navX.setAngleAdjustment(ang);
		//TODO: setAngleAdjustment doesn't seem to work.
		//navX.zeroYaw();
		//yawAngleBias = ang - (-navX.getAngle());
		mYawAngleBias = ang + mImu.getAngle();
		System.out.println("Angle zeroed to" + ang);
	}
	
	/**
	 * Get navX angle
	 * @return navX angle [deg]
	 */
	public double getAngle_Deg() {
		//NOTE: + angle = + rotation about robot z (up) axis
		//TODO: is there a native way to set the navx to an angle?
		return -mImu.getAngle() + mYawAngleBias;
	}
	
	
	/**
	 * Get navX yaw rate
	 * @return navX yaw rate [deg/sec]
	 */
	public double getRate_DPS() {
		//NOTE: + rate = robot counter-clockwise rotation (+ about robot z (up) axis)
		return -mImu.getRate();
	}
	
	public SwerveDrive getSwerveDrive() {
		return mDrive;
	}
	
	public double getCommandedRate() {
		return mRotationRateCmd;
	}
}
