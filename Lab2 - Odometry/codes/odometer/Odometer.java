/* This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import java.lang.Math;

/**
 * This class implements the Odometer for ECSE211 Lab 2 on the 
 * Lego EV3 platform.
 * 
 *  @author Julie Bellia, Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */


public class Odometer extends OdometerData implements Runnable {
  /**
   * <odoData> odometer data
   * <odo> returned as sngleton
   * <leftMotorTachoCount> distance of the left motor
   * <rightMotorTachoCount> distance of the right motor
   * <leftMotor> object of the left motor
   * <rightMotor> object of the right motor
   * <TRACK> difference between two wheels in centimeter
   * <WHEEL_RAD> radius of a wheel
   * <position> a double array storing current position
   * <ODOMETER_PERIOD> update rate in millisecond
   */
  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;
  private double[] position = new double[3];
  //private Object lock;
  //private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  //Display odometryDisplay = new Display(lcd);

  private static final long ODOMETER_PERIOD = 50; // odometer update period in ms
  
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    
    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    //lock = new Object();
  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");
    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    
    int pastMotorTachoCountL = leftMotorTachoCount;
    int pastMotorTachoCountR = rightMotorTachoCount;
    while (true) {
      updateStart = System.currentTimeMillis();
      
      // TODO Calculate new robot position based on tachometer counts
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();//TachoCount is the speed, isn't it?
      /*
      lcd.clear();
      lcd.drawString("Left Tacho = " + leftMotorTachoCount, 0, 0);
      lcd.drawString("Right Tacho = " + rightMotorTachoCount, 0, 0);
      try {
		Thread.sleep(100);
	  } catch (InterruptedException e) {
		}
      */
      //synchronized (lock) {
      double distL, distR, deltaD, dT, dX, dY, newT;
      position = odo.getXYT();
      
      distL = 2 * Math.PI * WHEEL_RAD * (leftMotorTachoCount - pastMotorTachoCountL) / 360; // computes wheel displacement
      distR = 2 * Math.PI * WHEEL_RAD * (rightMotorTachoCount - pastMotorTachoCountR) / 360; // 2*pi*r*(#wheelrotations)
      pastMotorTachoCountL = leftMotorTachoCount;
      pastMotorTachoCountR = rightMotorTachoCount;
      deltaD = 0.5 * (distL + distR);
      
      //dT = ((distL - distR) / TRACK) * (162/Math.PI);
      dT = Math.atan((distL - distR)/ TRACK) * (180/Math.PI);//In degreee
      newT = ((position[2] + (360 + dT) % 360) % 360) * (Math.PI/180);
      dX = deltaD * Math.sin(newT);
      dY = deltaD * Math.cos(newT);
      
      // TODO Update odometer values with new calculated values
      
    	  odo.update(dX, dY, dT);  
      //Sound.beep();
      //}
      // this ensures that the odometer only runs once every period
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
  

	

}
