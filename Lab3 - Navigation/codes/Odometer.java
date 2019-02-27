/* This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.lang.Math;

/**
 * This class implements the Odometer for ECSE211 Lab 3 on the 
 * Lego EV3 platform.
 * 
 *  @author Pengnan Fan, Julie Bellia
 *  
 */

public class Odometer extends Thread {
  
  //------<Important Constant>------//
  private final double TRACK;
  private final double RADIUS;
  private static final long ODOMETER_PERIOD = 50; // odometer update period in ms
  
  //------<Counter>------//
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  
  //------<Motors>------//
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  //------<Position>------//
  private static double[] position = new double[] {0,0,0};

  /**
   * This is the constructor of this class
   * 
   * @param leftMotor. The left motor of the robot
   * @param right motor. The right motor of the robot
   * @param RADIUS. This is the radius of a wheel in centimeter
   * @param TRACK. This is the distance between two wheels in centimeter
   */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double RADIUS, final double TRACK) {
	//Initialize the motors
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    
    //Initialize the counters
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    
    //Initialize the constants
    this.TRACK = TRACK;
    this.RADIUS = RADIUS;
  }

  /**
   * This is required by Thread.start(). It runs the odometer
   */
  public void run() {
    long updateStart, updateEnd;
    
    int pastMotorTachoCountL = leftMotorTachoCount;
    int pastMotorTachoCountR = rightMotorTachoCount;
    while (true) {
      updateStart = System.currentTimeMillis();
      
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      
      double distL, distR, deltaD, dT, dX, dY, newT;
      
      distL = 2 * Math.PI * RADIUS * (leftMotorTachoCount - pastMotorTachoCountL) / 360;
      distR = 2 * Math.PI * RADIUS * (rightMotorTachoCount - pastMotorTachoCountR) / 360;
      pastMotorTachoCountL = leftMotorTachoCount;
      pastMotorTachoCountR = rightMotorTachoCount;
      deltaD = 0.5 * (distL + distR);
      
      dT = Math.toDegrees(Math.atan((distL - distR)/ TRACK));//In degreee
      newT = ((position[2] + (360 + dT) % 360) % 360) * (Math.PI/180);
      dX = deltaD * Math.sin(newT);
      dY = deltaD * Math.cos(newT);
      
      position[0]+=dX;
      position[1]+=dY;
      position[2]+=dT;
      if(position[2]<0) {
    	position[2]+=360;
      }
      if(position[2]>=360) {
    	position[2]-=360;
      }
      
      Navigation.updatePosition(position);
      
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
      
    }
  }
  
  /**
   * This method returns current position recorded by odometer
   */
  public static double[] getPosition() {
	return position;
  }

	

}

