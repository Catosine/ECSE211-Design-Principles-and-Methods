package ca.mcgill.ecse211.lab3;

import java.text.DecimalFormat;
import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;

/**
 * This class implements the display for ECSE211 Lab 3 on the 
 * Lego EV3 platform.
 * 
 *  @author Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */

public class Display extends Thread{
  
  //------<Important Constant>------//
  private static final long DISPLAY_PERIOD = 50;
  
  //------<Navigation Infos>------//
  private static String map;
  private static String mode;
  
  //------<LCD>------//
  private static TextLCD SCREEN;
  
  //------<Position>------//
  private static double[] position = new double[] {0,0,0};
  
  /**
   * This is the class constructor
   * 
   * @param SCREEN. This is the lcd display
   * @param MODE. This is the index of navigation mode.
   * @param MAP. This is the index of MAP
   */
  public Display(TextLCD SCREEN, int MODE, int MAP) {
	if(MODE==Button.ID_LEFT) {
	  mode = "Simple";
	} else {
	  mode = "Obstacle";
	}
	if(MAP==Button.ID_UP) {
	  map = "1";
	}
	if(MAP==Button.ID_LEFT) {
	  map = "2";
	}
	if(MAP==Button.ID_RIGHT) {
	  map = "3";
	}
	if(MAP==Button.ID_DOWN) {
	  map = "4";
	}
    this.SCREEN = SCREEN;
  }

  /**
   * This is requried by Thread.start(). It runs the display
   */
  public void run() {   
	long updateStart, updateEnd;
	// Reset screen 
	SCREEN.clear();
	
	do {
      updateStart = System.currentTimeMillis();
      
      // Retrieve x, y and theta information
      position = Odometer.getPosition();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      SCREEN.drawString("<Mode> " + mode, 0, 0);
      SCREEN.drawString("<Map> " + map, 0, 1);
      SCREEN.drawString("<X> " + numberFormat.format(position[0]), 0, 2);
      SCREEN.drawString("<Y> " + numberFormat.format(position[1]), 0, 3);
      SCREEN.drawString("<Theta> " + numberFormat.format(position[2]), 0, 4);
		      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
		  Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
		} catch (InterruptedException e) {}
	  }
	} while (true);

  }
}
