package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implements the main for ECSE211 Lab 3 on the 
 * Lego EV3 platform.
 * 
 *  @author Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */

public class Lab3 {
  //------<Important Constant>------//
  private static int NAVI_MODE = 0;
  private static final double RADIUS = 2.1;//cm
  private static final double TRACK = 9.2;//cm
  private static final double TILE_SIZE = 30.48;//cm
  private static final int INFTY = 30;//cm
  private static final int MIN = 20;//cm
  private static final int MINIMUM  = 10;//cm
  private static final double[][] Xs =
    new double[][] {{0,1,2,2,1},{1,0,2,2,1},{1,2,2,0,1},{0,1,1,2,2}};//TILE_SIZE
  private static final double[][] Ys =
    new double[][] {{2,1,2,1,0},{1,2,2,1,0},{0,1,2,2,1},{1,2,0,1,2}};//TILE_SIZE
  
  //------<Motor>------//
  private static final EV3LargeRegulatedMotor LEFT_MOTOR =
    new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3LargeRegulatedMotor RIGHT_MOTOR =
    new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  
  //------<Odometer>------//
  private static final Odometer ODOMETER = new Odometer(LEFT_MOTOR, RIGHT_MOTOR, RADIUS, TRACK);
  
  //------<Navigation>------//
  private static final Navigation NAVI = new Navigation(LEFT_MOTOR, RIGHT_MOTOR, TILE_SIZE, MINIMUM, MIN, INFTY, RADIUS, TRACK);
  
  //------<LCD>------//
  private static final TextLCD SCREEN = LocalEV3.get().getTextLCD();
  
  //------<Display>------//
  private static Display DIS;
  
  /**
   * This is the main method of lab 3.
   * 
   * @param args
   */
  
  public static void main(String[] args) {
	int modeChoice, mapChoice;
	do {
	  // clear the display
	  SCREEN.clear();

	  // ask the user whether the robot go in simple mode or obstacle mode
	  SCREEN.drawString("<  Left | Right >", 0, 0);
	  SCREEN.drawString("        |        ", 0, 1);
	  SCREEN.drawString(" Simple |Obstacle", 0, 2);
	  SCREEN.drawString(" Mode   | Mode   ", 0, 3);
	  SCREEN.drawString("        |        ", 0, 4);

	  modeChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	} while (modeChoice != Button.ID_LEFT && modeChoice != Button.ID_RIGHT);
	
	do {
	  // clear the display
	  SCREEN.clear();

	  // ask the user to choose map
	  SCREEN.drawString("       <U>       ", 0, 0);
	  SCREEN.drawString("      Map 1      ", 0, 1);
	  SCREEN.drawString("<L>Map 2|Map 3<R>", 0, 2);
	  SCREEN.drawString("      Map 4      ", 0, 3);
	  SCREEN.drawString("       <D>       ", 0, 4);

	  mapChoice = Button.waitForAnyPress(); // Record choice
	} while (mapChoice != Button.ID_LEFT && mapChoice != Button.ID_RIGHT && mapChoice != Button.ID_DOWN && mapChoice != Button.ID_UP);
	
	SCREEN.clear();
	if(mapChoice==Button.ID_UP) {
	  NAVI_MODE = 0;
	}
	if(mapChoice==Button.ID_LEFT) {
	  NAVI_MODE = 1;
	}
	if(mapChoice==Button.ID_RIGHT) {
	  NAVI_MODE = 2;
	}
	if(mapChoice==Button.ID_DOWN) {
	  NAVI_MODE = 3;
	}
	
	DIS = new Display(SCREEN, modeChoice, mapChoice);
	//DIS.start();
	
	ODOMETER.start();
	if(modeChoice==Button.ID_LEFT) {
	  //Simple mode
	  NaiveController naive = new NaiveController(NAVI, NAVI_MODE, Xs, Ys);
	  naive.start();
	}
	if(modeChoice==Button.ID_RIGHT) {
	  //Obstacle mode
	  SensorController obstacle = new SensorController(NAVI, NAVI_MODE, Xs, Ys, INFTY);
	  obstacle.start();
	}
  }
}
