package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This is the main class for ECSE 211 Lab4 localization.
 * @author Pengnan Fan and Julie Bellia
 *
 */
public class Lab4 {
  
  //------<Important Constant>------//
  private static final double RADIUS = 2.1;//cm
  private static final double TRACK = 9.2;//cm
  private static final int SCANNING_SPEED = 50;
  private static final int ACCELERATION = 500;
  private static int LOCALIZE_MODE = 0;
  
  //------<Motor>------//
  private static final EV3LargeRegulatedMotor LEFT_MOTOR = 
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3LargeRegulatedMotor RIGHT_MOTOR = 
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  
  //------<Odometer>------//
  private static final Odometer ODOMETER = new Odometer(LEFT_MOTOR, RIGHT_MOTOR, RADIUS, TRACK);
  
  //------<LCD>------//
  private static final TextLCD SCREEN = LocalEV3.get().getTextLCD();
  
  //------<Display>------//
  private static Display DISPLAY;
  
  //------<UltrasonicLocalizer>------//
  private static UltrasonicLocalizer USLOCALIZER;
  
  //------<LightLocalizer>------//
  private static final LightLocalizer LIGHTLOCALIZER = new LightLocalizer(LEFT_MOTOR, RIGHT_MOTOR, RADIUS, TRACK) ;
  
  /**
   * This is the main method of localization
   * @param args
   */
  public static void main(String[] args) {
    int modeChoice;
	do {
	  // clear the display
	  SCREEN.clear();

	  // choosing: rising edge or falling edge
	  SCREEN.drawString("<  Left | Right >", 0, 0);
	  SCREEN.drawString("        |        ", 0, 1);
	  SCREEN.drawString(" Rising | Falling", 0, 2);
	  SCREEN.drawString(" Edge   | Edge   ", 0, 3);
	  SCREEN.drawString("        |        ", 0, 4);

	  modeChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	} while (modeChoice != Button.ID_LEFT && modeChoice != Button.ID_RIGHT);
	
	SCREEN.clear();
	if (modeChoice == Button.ID_LEFT) {
      LOCALIZE_MODE = 0;
	} else if (modeChoice == Button.ID_RIGHT) {
      LOCALIZE_MODE = 1;
	}
	
	ODOMETER.start();
	DISPLAY = new Display(SCREEN, LOCALIZE_MODE);
	DISPLAY.start();
	try {Thread.sleep(500);} catch (InterruptedException e1) {}
	
	USLOCALIZER = new UltrasonicLocalizer(LOCALIZE_MODE, LEFT_MOTOR, RIGHT_MOTOR, SCANNING_SPEED, ACCELERATION, RADIUS, TRACK);
	// Start of the localization procedure
	USLOCALIZER.localizeAngle();
	
	Sound.beep();
	
	// clear the display
	SCREEN.clear();

	SCREEN.drawString("                 ", 0, 0);
	SCREEN.drawString(" US Localization ", 0, 1);
	SCREEN.drawString("   is completed  ", 0, 2);
	SCREEN.drawString("                 ", 0, 3);
	SCREEN.drawString("                 ", 0, 4);
	
	try {Thread.sleep(5000);} catch (InterruptedException e) {}
	
	// clear the display
	SCREEN.clear();

	SCREEN.drawString("                 ", 0, 0);
	SCREEN.drawString("    Press any    ", 0, 1);
	SCREEN.drawString("     button      ", 0, 2);
	SCREEN.drawString("   to continue   ", 0, 3);
	SCREEN.drawString("                 ", 0, 4);

	modeChoice = Button.waitForAnyPress(); //Wait
	
	//Start light localization
	LIGHTLOCALIZER.findOrigin();
	
  }
}
