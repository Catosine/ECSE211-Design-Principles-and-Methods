package ca.mcgill.ecse211.wallfollowing;

/**
 * This class implements the bang-bang controller for Wall Follower for ECSE211 Lab 1 on the 
 * Lego EV3 platform.
 * 
 *  @author Julie Bellia, Pengnan Fan
 *  @debug Pengnan Fan
 *  @test Julie Bellia
 *  
 */

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {
	
	/**
	 * <FILTER_OUT> Maximum number of filter for unexpected distances, 50 by default
	 * <bandCenter> Distance between the ultrasonic sensor and walls
	 * <bandWidth> Maximum acceptable difference between measured distance and bandCenter
	 * <motorLow> Speed for moving slow
	 * <motorHigh> Speed for moving fast
	 * <distance> Distance measured by the ultrasonic sensor
	 * <filterCountrol> Current number of filter, 0 by default
	 * <INFTY> Infinity for the robot, 1.5 times of bandCenter by default
	 * <isIgnored> Boolean indicator for ignoring certain distance
	 * <minDistance> Minimum acceptable distance, 9 by default
	 */
	private static final int FILTER_OUT = 75;
	private final int bandCenter;
	private final int bandWidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl = 0;
	private final int INFTY;
	private boolean isIgnored;
	private int minDistance;
	
	/**
	 * This constructor assigns values to the data field and initializes motors of the EV3 
	 * robots.
	 * 
	 * @param bandCenter
	 * @param bandwidth
	 */
	public BangBangController(int bandCenter, int bandWidth, int motorLow, int motorHigh) {
		//Initializing data
		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.INFTY = (int)(bandCenter*1.5);
		this.minDistance =10;
		
		//Initializing motors
		WallFollowingLab.leftMotor.setSpeed(motorHigh);
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	};
	
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
		
		//Step 2: Move
	  	if(!isIgnored) {
	  		//Case 1: If the distance is NOT ignored, then proceed under given situations
	  		if(distance <= minDistance) {
	  			//Case 1.1: If the distance is below minDistance, then turn left while going 
	  			//          backward
	  			WallFollowingLab.leftMotor.setSpeed(45);
		  	    WallFollowingLab.rightMotor.setSpeed(75);
		  	    WallFollowingLab.leftMotor.backward();
		  	    WallFollowingLab.rightMotor.backward();
	  		} else if(distance >= INFTY) {
	  			//Case 1.2: If the distance is above INFTY, then turn left while going forwards
	  			WallFollowingLab.leftMotor.setSpeed(motorLow);
		  	    WallFollowingLab.rightMotor.setSpeed(motorHigh);
		  	    WallFollowingLab.leftMotor.forward();
		  	    WallFollowingLab.rightMotor.forward();
	  		} else if(distance>=bandCenter-bandWidth&&distance<=bandCenter+bandWidth) {
	  			//Case 1.3: If the distance is within the range of bandwidth, then go straight
	  			WallFollowingLab.leftMotor.setSpeed(motorHigh);
		  	    WallFollowingLab.rightMotor.setSpeed(motorHigh);
		  	    WallFollowingLab.leftMotor.forward();
		  	    WallFollowingLab.rightMotor.forward();
	  		} else if(distance>bandCenter+bandWidth) {
	  			//Case 1.4: If the distance is away from the wall, then turn left while going
	  			//          forward
	  			WallFollowingLab.leftMotor.setSpeed(motorLow);
		  	    WallFollowingLab.rightMotor.setSpeed(motorHigh);
		  	    WallFollowingLab.leftMotor.forward();
		  	    WallFollowingLab.rightMotor.forward();
	  		} else if(distance<bandCenter-bandWidth) {
	  			//Case 1.5: If the distance is close to the wall, then turn right while going
	  			//          forward
	  			WallFollowingLab.leftMotor.setSpeed(motorHigh);
		  	    WallFollowingLab.rightMotor.setSpeed(motorLow);
		  	    WallFollowingLab.leftMotor.forward();
		  	    WallFollowingLab.rightMotor.forward();
	  		}
	  	} else {
	  		//Case 2: If the distance is ignored, then go straight
	  		WallFollowingLab.leftMotor.setSpeed(motorHigh);
	  	    WallFollowingLab.rightMotor.setSpeed(motorHigh);
	  	    WallFollowingLab.leftMotor.forward();
	  	    WallFollowingLab.rightMotor.forward();
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
