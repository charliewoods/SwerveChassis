package org.usfirst.frc.team1592.robot.commands;

import org.usfirst.frc.team1592.robot.swerve.Discontinuities;
import org.usfirst.frc.team1592.robot.Constants;
import org.usfirst.frc.team1592.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoysticks extends Command {

    public DriveWithJoysticks() {
    	requires(Robot.chassis);
    }

    protected void initialize() {
    	//Default to field oriented mode
    	Robot.chassis.setFieldOriented(true);
    	Robot.chassis.setSetpoint(0.0);
    	//TODO: should we enable heading control here?
    	//Robot.chassis.enable();
    }

    protected void execute() {
    	//Get joystick commands
    	double xraw = Robot.oi.driver.getX(Hand.kLeft);
    	//Invert Y axis to make it + when stick is pushed forward
    	double yraw = -Robot.oi.driver.getY(Hand.kLeft);
    	//Apply exponential scaling on magnitude to give better sensitivity at low inputs
    	double mag = Discontinuities.joyExpo(Math.sqrt(xraw * xraw + yraw * yraw),4);
    	//Convert scaled magnitude back into x/y coordinates
    	double az = Math.atan2(yraw,xraw);
    	//Send commands to the swerve
    	//Note: SWERVE_RADIUS used to be diameter, so it's multiplied by 2 to keep the scaling like it was
    	Robot.chassis.driveSwerve(mag * Math.cos(az), mag * Math.sin(az), -Discontinuities.joyExpo(Robot.oi.driver.getU(), 4) / (Constants.SWERVE_RADIUS * 2));
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
    	end();
    }
}
