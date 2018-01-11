package org.usfirst.frc.team1592.robot.commands;

import org.usfirst.frc.team1592.robot.Constants;
import org.usfirst.frc.team1592.robot.Robot;

import org.usfirst.frc.team1592.robot.subsystems.RateLimiterND;
import org.usfirst.frc.team1592.robot.subsystems.XboxGamepad.AxisName;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoysticks_FPS extends Command {

	//Assume dt of 20ms
	//TODO: could swap over to a timer and measure dt
	private final static double dt = 0.02;		//sec
	
	//Last commands for smoothing purposes [output units/dt]
	//Set Max ramp rates (ramping disabled if set to zero)
	//A smaller ramp rate = slower changes in throttle
	private static double translateRamp = 2.5 * Constants.MAX_SPEED * dt; 	// [FPS/dt]
	private static double stopRamp = 1.0 * Constants.MAX_SPEED * dt; 		// [FPS/dt]
	private static double rotateRamp = 3.0 * Constants.MAX_TURN_RATE * dt; 	// [deg/s/dt]
	
	//======= Handling qualities =========
	private static final double kJoyDB = 0.1; //[ratio full scale]
	private static final double kCmdDB = 0.02;  //[ratio full scale]
	private static final double kJoyExpo = 3.0;
	
	//Rate Limiting
	private static boolean mIsRateLimiting = true; //default to on
	private static RateLimiterND lateralLimiter= new RateLimiterND(2,translateRamp,stopRamp);
	private static RateLimiterND rotateLimiter= new RateLimiterND(rotateRamp,rotateRamp/3.0);

    public DriveWithJoysticks_FPS() {
    	requires(Robot.chassis);
    }
    
    public static void setRateLimiting(boolean isRateLimiting) {
    	mIsRateLimiting = isRateLimiting;
    	if (isRateLimiting) {
    		//If enabling, let's reset states.  Would be better to use last if that was avail
    		lateralLimiter.setLastValue(0,0);
    		rotateLimiter.setLastValue(0);
    	}
    }
    
    /**
     * Set rate limits
     * NOTE: stop rate for rotational channel is harded coded to lateralStop / 3
     * @param translate rate limit for lateral velocity commands [cmd_units/s]
     * @param rotate rate limit for rotational velocity commands [cmd_units/s]
     * @param stop rate limit for lateral velocity commanded to 0 [cmd_units/s]
     */
    public static void setRamps(double translate, double rotate, double stop) {
    	lateralLimiter.setRateLimit(translate * dt);
    	lateralLimiter.setStoppingRateLimit(stop * dt);
    	rotateLimiter.setRateLimit(rotate * dt);
    	rotateLimiter.setStoppingRateLimit(rotate * dt / 3);
    }

    protected void initialize() {
    	//Default to field oriented mode
    	Robot.chassis.setFieldOriented(true);
    	//Set heading controller to current position and enable
    	//Robot.chassis.setSetpoint(Robot.chassis.getAngle());
    	//Robot.chassis.enable();
    	Robot.oi.driver.setCommandDeadband(kCmdDB);
    	Robot.oi.driver.setStickDeadband(kJoyDB);
    	Robot.oi.driver.setStickExpScale(kJoyExpo);
    }

    protected void execute() {
    	
    	double mag = Robot.oi.driver.getProcessedMagnitude(Hand.kLeft);
    	//Convert scaled magnitude back into x/y coordinates
    	double az = Robot.oi.driver.getDirectionRadians(Hand.kLeft);
    	double fwd = mag * Constants.MAX_SPEED * Math.cos(az);
    	double left = mag * Constants.MAX_SPEED * Math.sin(az);
    	//NOTE: Xbox ONE z axis likes to hang out at about 0.11
    	double ccw =  Constants.MAX_TURN_RATE * Robot.oi.driver.getProcessedAxis(AxisName.RIGHT_X);
    	
		//Smooth Commands
    	if (mIsRateLimiting) {
    		double[] outLat = lateralLimiter.limitRate(fwd,left);
    		double[] outRot = rotateLimiter.limitRate(ccw);

    		Robot.chassis.driveSwerve(outLat[0],outLat[1], outRot[0]);
    	} else {
    		Robot.chassis.driveSwerve(fwd,left, ccw);
    	}
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    	Robot.chassis.disable();
    }

    protected void interrupted() {
    	end();
    }
    
}