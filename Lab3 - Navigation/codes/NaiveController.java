package ca.mcgill.ecse211.lab3;

/**
 * This class implements the naive controller for ECSE211 Lab 3 on the 
 * Lego EV3 platform.
 * 
 *  @author Pengnan Fan
 *  @debug Pengnan Fan
 *  
 */

public class NaiveController extends Thread{
	
  public static Navigation NAVI;
  
  //------<Important Constants>------//
  private static double[][] Xs;
  private static double[][] Ys;
  private static int NAVI_MODE;
  
  /**
   * This is the constructor of this class
   * 
   * @param NAVI. This is the Navigation used to drive the robot
   * @param NAVI_MODE. This is the index of map (0 - 3)
   * @param Xs. This is an 2d-array of x coordinates
   * @param Ys. This is an 2d-array of y coordinates
   */
  public NaiveController(Navigation NAVI, int NAVI_MODE, double[][] Xs, double[][] Ys) {
	this.NAVI = NAVI;
	this.Xs = Xs;
	this.Ys = Ys;
	this.NAVI_MODE = NAVI_MODE;
  }
  
  /**
   * This is required by Thread.start(). It runs the naive controller
   */
  public void run() {
	for(int i = 0; i<5; i++) {
	  NAVI.travelTo(Xs[NAVI_MODE][i], Ys[NAVI_MODE][i]);
	}
	NAVI.turnTo(0);
  }
}
