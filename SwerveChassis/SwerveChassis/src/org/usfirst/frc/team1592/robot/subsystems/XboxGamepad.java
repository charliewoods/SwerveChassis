package org.usfirst.frc.team1592.robot.subsystems;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
Left Trigger (Back)                   Right Trigger (Back)

   _.-'BUMP `-._                          _,-'BUMP'-._
,-'             `-.,__________________,.-'      .-.    `-.
/      .-Y .                ___                ( Y )      \
/    ,' .-. `.      ____   / X \   _____    .-. `-` .-.    \
/   -X |   | +X    (Back) | / \ | (Start)  ( X )   ( B )   |
/    `. `-' ,'    __       \___/            `-` ,-. `-`    |
|      `+Y `   ,-`  `-.          .-Y .         ( A )       |
|             / -'  `- \       ,'  .  `.        `-`        |
|            |    POV   |     -X -  - +X                   |
!             \ -.  ,- /       `.  '  ,'                   |
|              `-.__,-'          `+Y `                     |
|                  ________________                        /
|             _,-'`                ``-._                  /
|          ,-'                          `-.              /  Based on 10/10 ASCII ART BYosrevad
\       ,'                                 `.           /
 `.__,-'                                     `-.______,'     */

public class XboxGamepad extends XboxController{
	
	//Threshold below which the signal is considered zero
	double m_stickDeadband = 0.1;
	//Minimum command to issue after exiting the stick deadband
	double m_commandDeadband = 0.02;
	// Exponential scaling to apply to the joysticks
	double m_joyExp = 3.0;
	
	public enum AxisName {
		LEFT_X(0),
		LEFT_Y(1),
		LEFT_TRIGGER(2),
		RIGHT_TRIGGER(3),
		RIGHT_X(4),
		RIGHT_Y(5);

		public final int value;

		private AxisName(int axis) {
			this.value = axis;
		}

	}
	
	
	public XboxGamepad(int port) {
		super(port);
	}
	
	/**
	 * + When pulled back on the right thumb stick
	 *
	 * @deprecated use {@link #getY(Hand.kRight)} instead.  
	 */
	@Deprecated
	public double getV() {
		return getY(Hand.kRight);
	}
	
	/**
	 * + When pushed right on the right thumb stick
	 *
	 * @deprecated use {@link #getX(Hand.kRight)} instead.  
	 */
	@Deprecated
	public double getU() {
		return getX(Hand.kRight);
	}
	
	  /**
	   * Get the button value (starting at button 1).
	   *
	   * <p>The appropriate button is returned as a boolean value.
	   * Button 11 treats the left trigger axis as a button.
	   * Button 12 treats the right trigger axis as a button
	   *
	   * @param button The button number to be read (starting at 1).
	   * @return The state of the button.
	   */
	@Override
	public boolean getRawButton(int button)
	{
		switch(button)
		{
			case 11:
				return getTriggerAxis(Hand.kLeft) > 0.1;
			case 12:
				return getTriggerAxis(Hand.kRight) > 0.1;
			default:
				return super.getRawButton(button);
		}
	}
	
	/**
	 * Get the magnitude of the direction vector formed by the joystick's current position relative to
	   * its origin.
	   *
	   * @param hand Side of controller whose value should be returned.
	   * @return The magnitude of the direction vector
	   */
	  public double getMagnitude(Hand hand) {
	    return Math.sqrt(Math.pow(getX(hand), 2) + Math.pow(getY(hand), 2));
	  }

	  /**
	   * Get the direction of the vector formed by the joystick and its origin in radians.
	   * 0 is forward. + angle is counter-clockwise
	   *
	   * @param hand Side of controller whose value should be returned.
	   * @return The direction of the vector in radians
	   */
	  public double getDirectionRadians(Hand hand) {
	    return Math.atan2(-getX(hand), -getY(hand));
	  }

	  /**
	   * Get the direction of the vector formed by the joystick and its origin in degrees.
	   *
	   * @param hand Side of controller whose value should be returned.
	   * @return The direction of the vector in degrees
	   */
	  public double getDirectionDegrees(Hand hand) {
	    return Math.toDegrees(getDirectionRadians(hand));
	  }
	  
	  /**
	   * Get the limited, mapped, and exp scaled axis value.  Y axes are inverted so
	   * that pushing the sticks forward return a positive value.
	   * @param name<AxisName>
	   * @return
	   */
	  public double getProcessedAxis(AxisName name) {
		  if (name.equals(AxisName.LEFT_Y) || name.equals(AxisName.RIGHT_Y)) {
			  //Invert y axes so that positive is stick pushed forward
			  return -processAxis(getRawAxis(name.value));
		  } else {
			  return processAxis(getRawAxis(name.value));
		  }
	  }
	  
	  /**
	   * Get the limited, mapped, and exp scaled axis value.  Y axes are inverted so
	   * that pushing the sticks forward return a positive value.
	   * @param hand Side of controller whose value should be returned.
	   * @return
	   */
	  public double getProcessedMagnitude(Hand hand) {
		  return processAxis(getMagnitude(hand));
	  }
	  
	  /**
	   * Axis processing consists of 3 steps: limiting, re-mapping, and exponentially scaling.
	   * 
	   * @param stickInput
	   * @return
	   */
	  double processAxis(double stickInput) {
		  
		  //Apply deadband
		  double deadCmd = dead(stickInput,m_stickDeadband);
		  //ensure the stick doesn't output mag > 1.0
		  double limitedCmd = limit(deadCmd, -1.0, 1.0);
		  //After leaving  the deadband, put on the linear range signum(cmd) * [stickDeadband,1.0]
		  double remappedCmd = map(limitedCmd,m_stickDeadband,m_commandDeadband);
		  //Apply exponential scaling on magnitude to give better sensitivity at low inputs
		  return scaleExponentially(remappedCmd, m_joyExp);
	  }
	  
		/**
		 * Clip an input to min and max 
		 * @param in Value in
		 * @param min Lowest value to allow
		 * @param max Highest value to allow
		 * @return The clipped value
		 */
		private static double limit(double in , double min, double max)
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
		 * @return Joystick value with deadband
		 */
		private static double dead(double in, double deadzone)
		{
			return Math.abs(in) > deadzone ? in : 0;
		}
		
		/**
		 * This function takes a joystick input and applies an exponential scaling
		 * For expo=1.5, the return value varies from about 50% of commanded at low inputs,
		 * To 80-100% of commanded at high rates
		 * @param stickInput Joystick in
		 * @param exp Exponential scale
		 * @return Scaled joystick value
		 */
		private static double scaleExponentially(double stickInput, double exp)
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
		
		
	    /**
	     * Maps input from magnitude [inLow,1] to [outLow,1]
	     * @param in
	     * @param inLow
	     * @param outLow
	     * @return value on output range
	     */
	    private static double map(double in, double inLow, double outLow) {
	    	double inRange = 1-inLow;
	    	double outRange = 1- outLow;
	    	double in_mag = (Math.abs(in) - inLow) * outRange/inRange + outLow;
	    	//Set to zero if < inLow
	    	if (in_mag < inLow) { in_mag = 0;}
	    	//Cap at maximum output = 1;
	    	if (in_mag > 1) { in_mag = 1;}
	    	return in_mag * Math.signum(in);
	    }
	    
	    public void setStickDeadband(double deadband){
	    	//Anything <= 0 disables
	    	m_stickDeadband = deadband;
	    }
	    
	    public void setCommandDeadband(double deadband){
	    	//Setting negative will do funny things.
	    	if (deadband >= 0) {
	    		m_commandDeadband = deadband;
	    	} else {
	    		m_commandDeadband = 0;
	    	}
	    }
	    
	    public void setStickExpScale(double exp){
	    	if (exp > 0) {
	    		m_joyExp = exp;
	    	} else {
	    		//A small value essentially disables. A value of zero would
	    		//create division by zero
	    		m_joyExp = 0.001;
	    	}
	    }
	
}
