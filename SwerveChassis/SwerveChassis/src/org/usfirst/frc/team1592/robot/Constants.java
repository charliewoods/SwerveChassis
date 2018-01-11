package org.usfirst.frc.team1592.robot;

import org.usfirst.frc.team1592.robot.swerve.CANTalonPID;
import org.usfirst.frc.team1592.robot.swerve.PIDConstants;
import org.usfirst.frc.team1592.robot.swerve.Vector2D;

public class Constants {
	
	public static final double SWERVE_WIDTH = 22.5d / 12d; //Front of robot [ft]
	public static final double SWERVE_LENGTH = 20.75d / 12d; //Side of robot [ft]
	public static final double SWERVE_RADIUS = Math.sqrt(SWERVE_WIDTH*SWERVE_WIDTH + SWERVE_LENGTH*SWERVE_LENGTH) / 2d; //[ft]
	public static final double WHEEL_RAD = 3.25 / 12d / 2d; //[ft]
	public static final double ENC2AXLE_RATIO = 24.0/64.0/1.6; //reduction from motor to encoder is 5:1.  Total reduction is 21.3333
	//TODO: need to maximize without going much over capability
	public static final double MAX_SPEED = 10.0;	//[FPS]
	//Note: max turn rate for chassis would be MAX_SPEED / SWERVE_RADIUS ~= 10FPS/1.275FT ~= 450 deg/s
	public static final double MAX_TURN_RATE = 1.0 * MAX_SPEED / SWERVE_RADIUS; //[rad/sec]
	//Minimum input speed to consider driving
	public static final double DRIVE_THRESH = 0.04 * 1.1; //[ratio of MAX_SPEED]
	
	//Swerve PIDs
	public static final CANTalonPID driveWheelPID = new CANTalonPID(0.1,0.0002,0.09,0.04,6000);
	public static final CANTalonPID rotatePodPID = new CANTalonPID(6.0,0.004,0.0);
	//Initialize the drive mode
	public static final boolean isVelDrive = true;
	
	private static final double RPM_enc2Kf(final double RPM_enc) {
		//Encoder is 4096 pulses per rev; Talon time unit is 100ms
		final double RPM2COUNTS_PER_100MS = 4096.0/10.0/60.0;
		//maximum talon output is integer 1023
		final double maxOut = 1023.0;
		//Feed forward gain: kF
		return maxOut / (RPM_enc * RPM2COUNTS_PER_100MS);
	}
	
	/**
	 * Helper method to convert wheel speed to a feed-forward gain
	 * @param RPM_wheel: RPM measured at wheel
	 * @return Kf: TalonSRX feed-forward gain
	 */
	private static final double RpmWheel2Kf(final double RPM_wheel) {
		return RPM_enc2Kf(RPM_wheel / ENC2AXLE_RATIO);
	}
	
	public final static class LFSwerve {
		public static final int DRIVE_CHAN = 15;  //Flight 15, Practice 0
		public static final int ROTATE_CHAN = 4; //Flight 4, Practice 1

		public static final double OFFSET = 0.133+0.5; //F: 0.133, P: 0.910
		public static final double Kf = RPM_enc2Kf(2845.8); //[talon_output/(counts/100ms)] TODO: derive for each wheel

		public static final Vector2D LOC = new Vector2D(SWERVE_LENGTH/2, SWERVE_WIDTH/2);
	}

	public static final PIDConstants TurnPID = new PIDConstants(Math.toRadians(2.5), 0.00015,0.05, 0d, 0.02);
	
	public final static class RFSwerve {
		public static final int DRIVE_CHAN = 0; //F: 0, P: 2
		public static final int ROTATE_CHAN = 11; //F: 11, P: 3
		
		public static final double OFFSET = 0.936+0.5; //F:0.936, P:0.763
		public static final double Kf = RPM_enc2Kf(3067.4); // [talon_output/(counts/100ms)] TODO: derive for each wheel

		public static final Vector2D LOC = new Vector2D(SWERVE_LENGTH/2, -SWERVE_WIDTH/2);
	}

	public final static class LBSwerve {
		public static final int DRIVE_CHAN = 14; //F: 14, P: 4
		public static final int ROTATE_CHAN = 5; //F: 5, P:5
		
		public static final double OFFSET = 0.926+0.5; //F:0.926 P:0.037
		public static final double Kf = RPM_enc2Kf(2866.6); //[talon_output/(counts/100ms)] TODO: derive for each wheel

		public static final Vector2D LOC = new Vector2D(-SWERVE_LENGTH/2, SWERVE_WIDTH/2);
	}

	public final static class RBSwerve {
		public static final int DRIVE_CHAN = 1; //F:1, P: 6
		public static final int ROTATE_CHAN = 10; //F: 10, P:7
		
		public static final double OFFSET = 0.358+0.5; //F:0.358, P: 0.690
		public static final double Kf = RPM_enc2Kf(2965.4); //[talon_output/(counts/100ms)] TODO: derive for each wheel

		public static final Vector2D LOC = new Vector2D(-SWERVE_LENGTH/2, -SWERVE_WIDTH/2);
	}

}
