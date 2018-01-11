package org.usfirst.frc.team1592.robot.swerve;

public class Vector2D {
	private double x, y;

	public Vector2D(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public void setXY(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public void set(Vector2D v) {
		this.x = v.x;
		this.y = v.y;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getMagnitude() {
		return Math.sqrt(x*x + y*y);
	}

	public double getAngleDeg() {
		return Math.toDegrees(getAngleRad());
	}
	
	public double getAngleRad() {
		return Math.atan2(y, x);
	}
	
	public Vector2D rotateByAngle(double angle_deg){
		double xo = x;
		double yo = y;
		double angle_rad = Math.toRadians(angle_deg);
		double xf =  Math.cos(angle_rad) * xo + Math.sin(angle_rad) * yo;
		double yf = -Math.sin(angle_rad) * xo + Math.cos(angle_rad) * yo;
		return new Vector2D(xf, yf);
	}

	public Vector2D add(Vector2D v) {
		return new Vector2D(x + v.x, y + v.y);
	}

	public Vector2D sub(Vector2D v) {
		return new Vector2D(x - v.x, y - v.y);
	}
	
	public Vector2D scale(double scale) {
		return new Vector2D(x * scale, y * scale);
	}
	
	public Vector2D scale(double scaleX, double scaleY) {
		return new Vector2D(x * scaleX, y * scaleY);
	}
	
	/**
	 * Computes the cross product assuming the z component of the
	 * first vector is 0, and the x and y components of the second
	 * vector are 0.  Therefore the x and y components of the resulting
	 * vector are (-y*z,x*z)
	 * @param z component of vector 1
	 * @return V1 x V2
	 */
	public Vector2D cross2D(double z) {
		double x = -this.y * z;
		double y =  this.x * z;
		return new Vector2D(x, y);
	}
	
	/**
	 * Dot Product of 2D vector
	 * @param v <Vector2D>
	 * @return dot product
	 */
	public double dot(Vector2D v) {
		return v.x * x + v.y * y;
	 	}
	
	@Override
	public String toString() {
		return getClass().getSimpleName()+": ("+x+", "+y+") = "+Math.floor(getMagnitude())+"@"+Math.floor(getAngleDeg()) + "deg";
	}
}