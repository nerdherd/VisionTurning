
package org.usfirst.frc.team687.robot;

import com.kauailabs.navx_mxp.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    
	NetworkTable table;
	
	VictorSP ftLeft;
	VictorSP ftRight;
	VictorSP bkLeft;
	VictorSP bkRight;
	
	AHRS nav;
	
	double kP = 0.0044444;
	double camWidth = 320;
    
    public void robotInit()	{
        table = NetworkTable.getTable("GRIP/myContoursReport");
    	
        ftLeft = new VictorSP(0);
        ftRight = new VictorSP(3);
        bkLeft = new VictorSP(1);
        bkRight = new VictorSP(4);
        
        nav = new AHRS(new SerialPort(57600, SerialPort.Port.kMXP));
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	// Get the Contour Report
    	double[] emptyVal = new double[0];
        double[] centerX = table.getNumberArray("centerX", emptyVal);
        double[] area = table.getNumberArray("area", emptyVal);
        
        double pow = 0;
        
        if(centerX != emptyVal)	{
        	
        	// Find the biggest contour
        	int max = 0;
        	for(int i = 0; i < area.length; i++)	{
        		max = area[i] > max ? i : max;
        	}
        	
        	double desired = camWidth/2;
        	double error = desired-centerX[max];
        	pow = kP * error;
        }	else	{
        	pow = 0;
        }
        
        SmartDashboard.putNumber("Power", pow);
        
        ftLeft.set(pow);
        ftRight.set(pow);
        bkLeft.set(pow);
        bkRight.set(pow);
    }
    
}
