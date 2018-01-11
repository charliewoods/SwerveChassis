package org.usfirst.frc.team1592.robot.swerve;

import java.util.ArrayList;
import java.util.Arrays;

import org.usfirst.frc.team1592.robot.swerve.Vector2D;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

public class SwerveDrive implements LiveWindowSendable {
	//Define physical devices
	ArrayList<SwervePod> pods = new ArrayList<SwervePod>();
	//Collect commands in an array in case they need retrieval for tasks like logging
	ArrayList<Vector2D> podCmds = new ArrayList<Vector2D>();
	
	//Set max velocity of a wheel (default = full scale ratio)
	private double vmax = 1;
	//Scale factor from input command to drive wheel command. see setter for details
	private double input2DriveCmd = 1.0;
	//vel threshold to command a pod to an angle
	private double kVelThresh = 0.04; //default to 4%
	//Last commanded velocities
	private Vector2D mBodyVelocityCmd = new Vector2D(0,0);
	private double mBodyRateCmd = 0;
	
	/**
	 * Creates a new swerve drive object
	 * @param SwervePod objects
	 */
	public SwerveDrive( SwervePod... pods) {
		//Pack all pods in the array
		this.pods.addAll(Arrays.asList(pods));
		//initialize all commands to zero
		for(int i=0; i<pods.length; i++) {
			podCmds.add(i, new Vector2D(0, 0));
		}
	}
	
	/**
	 * Changes the talon control mode for all of the pod drive motors
	 *
	 * @param mode  the new mode
	 */
	public void changeDriveControlMode(TalonControlMode mode) {
		for (SwervePod pod : pods) {
			pod.changeDriveControlMode(mode);
		}
	}
	
	/**
	 * Establishes the drive motor profiles for all of the pod drive motors.
	 */
	public void establishDriveProfiles() {
		for (SwervePod pod : pods) {
			pod.establishDriveProfiles();
		}
	}
	
	/**
	 * Sets the drive profiles on all the pods to the specified slot.
	 *
	 * @param slot  PID parameter slot on the Talon
	 */
	public void setDriveProfiles(int slot) {
		for (SwervePod pod : pods) {
			pod.setDriveProfile(slot);
		}
	}
	
	/**
	 * Drive using joystick commands around a pivot point
	 * Commands are interpreted in the body-fixed frame
	 * @param fwd Translation (+ forward)
	 * @param left Translation (+ left)
	 * @param ccw Rotation cmd (+ counter-clockwise)
	 * @param pivotPoint - point from robot center to rotate about
	 * 	 */
	public synchronized void drive(final double fwd, final double left, final double ccw, Vector2D pivotPoint) {
		
		//Compute individual pod commands
		mBodyVelocityCmd = new Vector2D(fwd, left);
		mBodyRateCmd = ccw;
		//Reset register command normalization
		double largestCmd = 0;
    	//Loop to compute cmd to each pod
		for(int i=0; i<podCmds.size(); i++) {	    	
			//Get Distance to pod from pivot point (usually center of chassis)
			Vector2D pivot2Pod = pods.get(i).getPosition().sub(pivotPoint);
			//Velocity of the pod is the velocity of the pod plus extra velocity from rotation:
			//v_pod = v_body + w x r
			Vector2D velocityCmd = mBodyVelocityCmd.add(pivot2Pod.cross2D(ccw));
			//Track maximum commanded wheel speed
			largestCmd = velocityCmd.getMagnitude() > largestCmd? velocityCmd.getMagnitude():largestCmd;
			//Collect commands in a vector
			podCmds.set(i, velocityCmd);
		}
		
		//Normalize commands to max output
		if (largestCmd > vmax) {
			for(int i = 0; i < podCmds.size(); i++) {
				//vmax/largestMag will set largest to vmax and scale the others by the same ratio
				podCmds.get(i).set(podCmds.get(i).scale(vmax/largestCmd));
			}
			//Get scaled command after normalization (can't do with a single pod, but why would you have 1 pod?)
			//NOTE: this is really only needed if ramping is being done.  Now that ramping was moved up to SwerveSubsystem,
			//the new command would need to be set to a field and made publicly accessible
			if (pods.size() > 1) {
				SwervePod pod0 = pods.get(0);
				SwervePod pod1 = pods.get(1);
				//Get the relative position between the 2 pods
				Vector2D r = pod1.getPosition().sub(pod0.getPosition());
				//Get the relative velocity between the 2 pods
				Vector2D vrel = podCmds.get(1).sub(podCmds.get(0));
				//Compute rotation rate from relative positions and velocities
				//Have to check to make sure that the x or y coordinates of the 2 pods aren't the same
				//Pods can't physically be at the same spot but software could be (incorrectly) set that way
				if (r.getX() > 0) {
					mBodyRateCmd = vrel.getY() / r.getX();
				} else if (r.getY() > 0) {
					mBodyRateCmd = -vrel.getX() / r.getY();
				}
				//v_body = v_pod - w x r (use first pod because it doesn't matter which one)
				mBodyVelocityCmd = podCmds.get(0).sub(pods.get(0).getPosition().sub(pivotPoint).cross2D(mBodyRateCmd));
			}
		}
    	
    	//Send the command to each pod
    	for(int i=0; i < pods.size(); i++) {
    		//Get pod from the array
    		SwervePod pod = pods.get(i);
    		//Get command from the array
    		Vector2D podCmd = podCmds.get(i);
    		//Send the command to the pod in polar form
    		double cmdMag = podCmd.getMagnitude();
    		double cmdAz_deg = podCmd.getAngleDeg();
			//Threshold output
    		if(Math.abs(cmdMag) > kVelThresh * vmax) {
    			//Convert from input to output units
    			cmdMag *= input2DriveCmd;
    		} else {
    			//Command is too small to do anything with
        		//TODO: should we rotate the pod at the rate the chassis is rotating if below threshold?
    			cmdMag = 0.0;
    			//Command current position
    			cmdAz_deg = pod.getWheelAngle_Deg();
    		}
    		pod.set(cmdAz_deg, cmdMag);
    	}
	}
	
	/**
	 * Get the last issue commands in the body fixed frame
	 * @return cmds<double[]> array of {fwd,left,ccw} commands 
	 */
	public synchronized double[] getLastCommands() {
		return new double[] {mBodyVelocityCmd.getX(),mBodyVelocityCmd.getY(),mBodyRateCmd};
	}
	
	/**
	 * Estimate the velocity of the center of the robot by averaging
	 * the estimated velocity from each pod
	 * @param thetaDot_rps chassis rotation rate in rad/sec
	 * @return Vector velocity of the robot in the field frame
	 */
	public Vector2D getVelocityBody_FPS(double thetaDot_rps) {
		int nPods = pods.size();
		double[] vx = new double[nPods];		//x velocity of each pod
		double[] vy = new double[nPods];		//y velocity of each pod
		Vector2D vPod; 							//velocity of pod
		Vector2D rPod;							//pod location on the robot
		for(int i=0; i<nPods; i++) {
			vPod = pods.get(i).getVelocity_FPS();
			rPod = pods.get(i).getPosition();
			//Note: use this for median
			vx[i] = vPod.getX() + thetaDot_rps * rPod.getY();
			vy[i] = vPod.getY() - thetaDot_rps * rPod.getX();
		}
		
		//Select median velocity since we have more than enough measurements
		//Better to take median? -> Less prone to outliers
		//Note: should really take median of a wheel - not the individual measurements? but geometric median is hard
		//TODO: Do some fault detection (find and discard outliers?)
		double xMedian, yMedian;
		Arrays.sort(vx);
		Arrays.sort(vy);
		if (nPods % 2 == 0) {
			//Even number of pods -> take average of middle 2
			xMedian = (vx[nPods/2 - 1] + vx[nPods/2]) / 2;
			yMedian = (vy[nPods/2 - 1] + vy[nPods/2]) / 2;
		} else {
			//Odd number of pods -> take middle (integer division will round down)
			xMedian = vx[nPods/2 + 1];
			yMedian = vy[nPods/2 - 1];
		}
		//Call anything less than ~3bits zero.
//		if (Math.abs(xMedian) < 1e-3) xMedian = 0;
//		if (Math.abs(yMedian) < 1e-3) yMedian = 0;
		//return estimated chassis velocity in the body fixed frame
		return new Vector2D(xMedian,yMedian);
		
	}
	
	/**
	 * Set minimum wheel command that will allow commands to rotation motor
	 * @param lim Ratio of maximum velocity command
	 */
	public void setPodRotateVelThreshold(double lim) {
		//Force limit to be positive
		if (lim < 0) {lim = -lim;}
		kVelThresh = lim;
	}
	
	/**
	 * Get minimum wheel command that will allow commands to rotation motor
	 * @param kVelThresh Ratio of maximum velocity command
	 */
	public double getPodRotateVelThreshold() {
		return kVelThresh;
	}
	
	/**
	 * Get the limiting value on wheel speed
	 * @return vmax in wheel command units
	 */
	public double getMaxAllowableWheelSpeed() {return vmax;}
	
	/**
	 * Set the maximum speed output a pod can be commanded at 
	 * @param v
	 */
	public void setMaxAllowableWheelSpeed(double v) {vmax = v;}
		
	/**
	 * Get the scale factor to convert swerve drive input to motor cmd
	 * @return the input2DriveCmd
	 */
	public double getInput2DriveCmdScale() {
		return input2DriveCmd;
	}

	/**
	 * Set the scale factor to convert swerve drive input to motor cmd
	 * Ex: if commands are issued in ft/sec, but pod is commanded in encoder RPM
	 * input2DriveCmd = 1/ radiusWheel / kRPM2RPS / enc2whlRatio
	 * @param input2DriveCmd
	 */
	public void setInput2DriveCmdScale(double input2DriveCmd) {
		//Update the scale factor
		this.input2DriveCmd = input2DriveCmd;
	}

	private ITable lwTable;

	@Override
	public void initTable(ITable subtable) {
		this.lwTable = subtable;
	}

	@Override
	public ITable getTable() {
		return lwTable;
	}

	@Override
	public String getSmartDashboardType() {
		return "Swerve Drive";
	}

	@Override
	public void updateTable() {
		if(lwTable != null) {
			for(int i=0; i < pods.size(); i++) {
				SwervePod pod = pods.get(i);
				lwTable.putNumber("Swerve Pod "+i, pod.getAbsAngle());
			}
		}
	}

	@Override
	public void startLiveWindowMode() {}

	@Override
	public void stopLiveWindowMode() {}
}