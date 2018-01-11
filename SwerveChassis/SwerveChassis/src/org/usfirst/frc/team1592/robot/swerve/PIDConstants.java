package org.usfirst.frc.team1592.robot.swerve;

import org.usfirst.frc.team1592.robot.swerve.CANTalonPID;

public class PIDConstants {
	//kF: Feed-forward		[outputUnits/errorUnits]
	//kP: Proportional 		[outputUnits/errorUnits]
	//kI: Integral 			[outputUnits / (errorUnits * period)]
	//kD: Derivative		[outputUnits / (errorUnits / period)]
	//iZone: I-Zone			[outputUnits]
	//period: Loop period	[s]
	public double kP, kI, kD, kF;
	public double period = 0.02;
	public int iZone;
	
	public PIDConstants(double kP, double kI, double kD, double kF) {
		this(kP, kI, kD, kF, 0);
	}
	
	public PIDConstants(double kP, double kI, double kD, double kF, double period) {
		this(kP, kI, kD, kF, 0,period);
	}
	
	public PIDConstants(double kP, double kI, double kD, double kF, int iZone) {
		this(kP, kI, kD, kF, iZone, 0.02);
	}

	public PIDConstants(double kP, double kI, double kD, double kF, int kIz,double period) {
		this.kF = kF;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.iZone = kIz;
		this.period = period;
	}
	
	/**
	 * 
	 * @param fullScaleOutput
	 * @param rotateUnits2Revs
	 * @return
	 */
	public CANTalonPID toTalonSrxVelocityConstants(double fullScaleOutput,double rotateUnits2Revs) {
		//outputUnits = 1023 [TalonThrottle / Full Scale Out]
		final double WPI_2_TALON_OUTPUT = 1023 / fullScaleOutput;
		//Native units for Mag encoders are counts/100ms
		final int COUNTS_PER_REV = 4096;
		final int HUNDRED_MS_PER_S = 10;
		final double WPI_ERR_UNITS_2_TALON_NATIVE_UNITS = rotateUnits2Revs / (COUNTS_PER_REV/HUNDRED_MS_PER_S);
		//TalonSRX control loop runs at 1kHz, so we have to convert the period for integral and derivative
		final double THOUSAND_MS_PER_PERIOD = 1000 * this.period;
		//nativeUnits = counts/100ms
		//kF: Feed-forward 	[TalonThrottle/nativeUnits]
		//kP: Proportional	[TalonThrottle/nativeUnits]
		//kI: Integral		[TalonThrottle/nativeUnits/1ms]
		//kD: Derivative	[TalonThrottle/nativeUnits*1ms]
		//kIz: I-Zone 		[native units]
		double kF = this.kF * WPI_2_TALON_OUTPUT / WPI_ERR_UNITS_2_TALON_NATIVE_UNITS;
		double kP = this.kP * WPI_2_TALON_OUTPUT / WPI_ERR_UNITS_2_TALON_NATIVE_UNITS;
		double kI = this.kI * WPI_2_TALON_OUTPUT / WPI_ERR_UNITS_2_TALON_NATIVE_UNITS / THOUSAND_MS_PER_PERIOD;
		double kD = this.kD * WPI_2_TALON_OUTPUT / WPI_ERR_UNITS_2_TALON_NATIVE_UNITS * THOUSAND_MS_PER_PERIOD;
		//NOTE: direct conversion between PIDController1592 iZone to TalonSRX iZone isn't possible because WPI
		//cages the total output while Talon cage the error signal.  The below conversion assumes feedback, integral
		//and derivative states are all zero = initial step
		int iZone = (int) (this.iZone * WPI_2_TALON_OUTPUT / (kP+kF));
		return new CANTalonPID(kP,kI,kD,kF,iZone);
	}
	
	//TODO: toTalonSrxPositionConstants?
}
