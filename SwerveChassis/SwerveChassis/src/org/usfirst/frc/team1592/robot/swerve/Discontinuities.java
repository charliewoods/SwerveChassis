package org.usfirst.frc.team1592.robot.swerve;

public class Discontinuities
{
	/**
	 * Keep heading commands between 0 and 360
	 * @param angle_deg: Angle to be wrapped
	 * @return Wrapped angle
	 */
	public static double wrapAngle0To360Deg(double angle_deg) {
		angle_deg = angle_deg % 360.0;  
		if (angle_deg < 0) angle_deg += 360.0;
		return angle_deg;
	}
	
	/**
	 * Clip an input to min and max but preserve sign
	 * @param in Value in
	 * @param min Lowest value to allow
	 * @param max Highest value to allow
	 * @return The clipped  value
	 */
	public static double limitMagnitude(double in , double min, double max)
	{
		double out = Math.abs( in ); //Remove sign for bounds testing
		if(out > max)
		{
			out = max;
		}
		if(out < min)
		{
			out = min;
		}
		return out * Math.signum( in ); //Re-add sign
	}
	
	/**
	 * Clip an input to min and max 
	 * @param in Value in
	 * @param min Lowest value to allow
	 * @param max Highest value to allow
	 * @return The clipped value
	 */
	public static double limit(double in , double min, double max)
	{
		double out = in;
		if(out > max)
		{
			out = max;
		}
		if(out < min)
		{
			out = min;
		}
		return out;
	}
	
	/**
	 * Deadzone filter
	 * @param in Joystick value in
	 * @param deadband Deadband
	 * @return Joystick value with deadband
	 */
	public static double dead(double in , double deadband)
	{
		return Math.abs(in) > deadband ? in : 0;
	}
	
	/**
	 * Returns true if a in is in the range of [min, max]
	 * @param in Value in
	 * @param min Minimum value
	 * @param max Maximum value
	 * @return true if in is between [min, max]
	 */
	public static boolean isBetween(double in, double min, double max)
	{
		return (min < in && in < max)?true:false;
	}

	/**
	 * This function takes a joystick input and applies an exponential scaling
	 * For expo=1.5, the return value varies from about 50% of commanded at low inputs,
	 * To 80-100% of commanded at high rates
	 * @param stickInput Joystick in
	 * @param exp Exponential scale
	 * @return Scaled joystick value
	 */
	public static double joyExpo(double stickInput,double exp)
	{
		double stickOutput;
		
		//Convert linear input magnitude to exponential from 0
		stickOutput = Math.exp(Math.abs(stickInput) * exp) -1;
		//Normalize to unit magnitude
		stickOutput = stickOutput / (Math.exp(exp) - 1);
		//Reapply polarity
		stickOutput = Math.signum(stickInput) * stickOutput;
		
		return stickOutput;
	}
	
	public static int sign(double num)
	{
		return (int)(Math.signum(num)==0?1:Math.signum(num));
	}
}