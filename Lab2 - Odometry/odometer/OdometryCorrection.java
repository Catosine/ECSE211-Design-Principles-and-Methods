/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Display;
import java.io.File;
import lejos.hardware.Audio;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.LinkedList;
  /**
   * This class implements the Odometer with correction for ECSE211 Lab 2 on the 
   * Lego EV3 platform.
   * 
   *  @author Julie Bellia, Pengnan Fan
   *  @debug Pengnan Fan
   *  
   */
public class OdometryCorrection implements Runnable {
  
  /**
   * <CORRECTION_PERIOD> rate of correction in millisecond
   * <AVG_COUNTER> number of filter for light sensor
   * <lsPort> port for light sensor
   * <light> sample provider for light sensor
   * <lightSensor> sensor modes for the light sensor
   * <odometer> odometer
   * <lightData> an array of float storing data from light sensor
   * <pos> an array of double storing position data
   * <TILE_SIZE> a constant for the length of a side of a tile
   */
	
  private static final long CORRECTION_PERIOD = 50;
  private static final int AVG_COUNTER = 5;
  
  private static Port lsPort = LocalEV3.get().getPort("S1");
  private static SampleProvider light;
  private static SensorModes lightSensor;
  private Odometer odometer;
  private float[] lightData;
  //private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  //private Display odometryDisplay = new Display(lcd); 
  private static double[] pos = new double[3];
  private static final double TILE_SIZE = 30.48;
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @param odo
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection(Odometer odo) throws OdometerExceptions {
	
    this.odometer = odo;
    
    this.lightSensor = new EV3ColorSensor(lsPort);
    //this.angleSensor = new EV3GyroSensor(asPort);
    this.light = ((EV3ColorSensor)lightSensor).getRedMode();
    //this.angle = ((EV3GyroSensor)angleSensor).getAngleMode();
    
    this.lightData = new float[lightSensor.sampleSize()];
    //this.angleData = new float[angleSensor.sampleSize()];
    //this.odometer = Odometer.getOdometer();
   
    //System.out.println(odoData==null);
    Sound.setVolume(100);
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  	public void run() {
  		long correctionStart, correctionEnd;//Time counted
  		odometer.setXYT(-15.24, -15.24, 0);
  		double sumLight = 0;
  		double avgLight = 0;
  		int countX = 0;
  		int countY = 0;
  		double ratio = 0;
  		int counter = 0;
  		double STANDARD = 0;
  		for(int i = 0; i<100; i++) {
  			light.fetchSample(lightData, 0);
  			STANDARD+=lightData[0]*100;
  		}
  		STANDARD/=100;
  		while (true) {
  			correctionStart = System.currentTimeMillis();
  		
  			// TODO Trigger correction (When do I have information to correct?)
  			//Avg light filter
  			for(int i = 0; i<AVG_COUNTER; i++) {
  				light.fetchSample(lightData, 0);
  				sumLight+=lightData[0]*100;
  			}
  			// TODO Update odometer with new calculated (and more accurate) vales
  			
  			avgLight = (sumLight+0.0)/(AVG_COUNTER+0.0);
  			ratio = (avgLight/STANDARD);
  			//System.out.println(avgLight);
  			if((ratio>=1.15||ratio<=0.85)&&counter<12) {
  				Sound.beep();
  				pos = odometer.getXYT();
  				counter++;
  			}
  			
  			if(counter!=1&&counter!=4&&counter!=7&&counter!=10&&counter<=12) {
  				double deg = pos[2];
  				if(deg>=355||deg<=5) {
  					countY++;
  					odometer.setY(countX*TILE_SIZE);
  				}
  				if(deg>=85&&deg<=95) {
  					countX++;
  					odometer.setX(countY*TILE_SIZE);
  					
  				}
  				if(deg>=175&&deg<=185) {
  					countY--;
  					odometer.setY(countX*TILE_SIZE);
  				}
  				if(deg>=265&&deg<=275) {
  					countX--;
  					odometer.setX(countY*TILE_SIZE);
  				}
  			}
  			if(counter==1) {
  				odometer.setY(0);
  			}
  			if(counter==4) {
  				odometer.setX(0);
  			}
  			if(counter == 9) {
  				odometer.setY(0);
  			}
  			if(counter == 12) {
  				odometer.setX(0);
  			}
  			
  			//lcd.clear();
  			//lcd.drawString("Light = " + avgLight, 0, 0);
  			
  			//Reset
  			avgLight = 0;
  			sumLight = 0;
  			ratio = 0;
  			
  			// this ensure the odometry correction occurs only once every period
  			correctionEnd = System.currentTimeMillis();
  			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
  				try {
  					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
  				} catch (InterruptedException e) {
  					// there is nothing to be done here
  				}
  			}
  		}
  	}	
}
