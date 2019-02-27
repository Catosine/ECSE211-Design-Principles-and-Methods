package ca.mcgill.ecse211.wallfollowing;

/**
 * This class implements the P-controller for Wall Follower for ECSE211 Lab 1 on the Lego EV3
 * platform.
 * 
 *  @author Pengnan Fan
 *  @debug Pengnan Fan
 *  @test Julie Bellia
 *  
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/**
	 * <MOTOR_SPEED> Speed for motors, 135 by default
	 * <FILTER_OUT> Maximum number of filter for unexpected distances, 20 by default
	 * <leftSpeed> Speed for left motor, MOTOR_SPEED by default
	 * <rightSpeed> Speed for right motor, MOTOR_SPEED by default
	 * <bandCenter> Distance between the ultrasonic sensor and walls
	 * <bandWidth> Maximum acceptable difference between measured distance and bandCenter
	 * <diffRatio> The ratio between bandWidth and bandCenter
	 * <distance> Distance measured by the ultrasonic sensor
	 * <filterCountrol> Current number of filter, 0 by default
	 * <INFTY> Infinity for the robot, 1.5 times of bandCenter by default
	 * <isIgnored> Boolean indicator for ignoring certain distance
	 * <minDistance> Minimum acceptable distance, 9 by default
	 */
	
	private static final int MOTOR_SPEED = 150;
	private static final int FILTER_OUT = 50;
	private int leftSpeed = MOTOR_SPEED;
	private int rightSpeed = MOTOR_SPEED;
	private final int bandCenter;
	private final int bandWidth;
	private final double diffRatio;
	private int distance;
	private int filterControl;
	private final int INFTY;
	private boolean isIgnored;
	private final int minDistance;
	
	/**
	 * This constructor assigns values to the data field and initializes motors of the EV3 
	 * robots.
	 * 
	 * @param bandCenter
	 * @param bandwidth
	 */
	public PController(int bandCenter, int bandwidth) {
		//Initializing data
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		diffRatio = (bandWidth+0.0)/bandCenter;
		this.filterControl = 0;
		this.INFTY = (int)(bandCenter*1.5);
		this.minDistance = 9;
		
		//Initializing motors
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	}
	
	/**
	 * This processUSData method runs after users selecting control mode. It allows the robot to
	 * take different moves based on the input.
	 * 
	 * @param distance: distance measured by the ultrasonic sensor
	 */
	@Override
	public void processUSData(int distance) {
		//Step 1: Filter
		isIgnored = false;
		if (distance >= INFTY && filterControl < FILTER_OUT) {
			//Case 1: Filter out a certain distance and set it to be ignored
			filterControl++;
			isIgnored = true;
		} else if (distance >= INFTY) {
			//Case 2: Filter is full. Take the actual distance and set it to be 2 times of 
			//        bandCenter
			this.distance = bandCenter*2;
		} else {
			//Case 3: Reset the filter
			filterControl = 0;
			this.distance = distance;
		}
		
		//Step 2: Calculate the ratio between the measured distance and the bandCenter
		double ratio = ((this.distance+0.0)/bandCenter) - 1;
		
		if(ratio>=(-diffRatio)&&ratio<=diffRatio) {
			//Case 1: If the ratio is within a range [-diffRatio, diffRatio], then ignore 
			//        this measurement
			isIgnored = true;
			ratio = 0;
		}
		
		//Step 3: Calculate the factor for changing speed
		double factor = 1 + Math.abs(ratio);// Maximum = 5/3
		
		if(factor>(10.0/6)) {
			//Case 1: If the factor is greater than 5/3, then set it to be 5/3.
			factor = (10.0/6);
		}
		
		//Step 4: Calculate speed for motors
		if(ratio>0) {
			//Case 1: The robot is too far from the wall
			leftSpeed = MOTOR_SPEED;
			rightSpeed = (int)(MOTOR_SPEED * factor);
		} else if(ratio<0) {
			//Case 2: The robot is too close to the wall
			leftSpeed = (int)(MOTOR_SPEED  * factor);
			rightSpeed = MOTOR_SPEED;
		} else {
			//Case 3: Default case
			leftSpeed = MOTOR_SPEED;
			rightSpeed = MOTOR_SPEED;
		}
		
		//Step 5: Move
		if(distance<minDistance) {
			//Case 1: If the robot is too close to the wall, then turn left while going backward
			WallFollowingLab.leftMotor.setSpeed(45);
			WallFollowingLab.rightMotor.setSpeed(75);
			WallFollowingLab.leftMotor.backward();
			WallFollowingLab.rightMotor.backward();
		} else {
			if(!isIgnored) {
				//Case 2: If the distance is NOT ignored, then move at calculated speed
				WallFollowingLab.leftMotor.setSpeed(leftSpeed);
				WallFollowingLab.rightMotor.setSpeed(rightSpeed);
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.forward();
			} else {
				//Case 3: If the distance is ignored, then go straight at default speed
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.forward();
			}
		}
		
	}

	/**
	 * This readUSDistance method takes no inputs but return this.distance.
	 * 
	 * @param No input
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
