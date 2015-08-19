/*****************************************************************************
 * WindowManager.java
 * 
 * Creates the GUI and maintains all aspects of it
 *  
 ****************************************************************************/

package rov.gui;

import rov.gui.window.*;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;

public class WindowManager
{
    // Window Attributes
    private final int FRAME_W = 1000, FRAME_H = 500;
    private final String TITLE = "ROV Controller";
    
    // Window Components
    private ArrayList<JInternalFrame> windows;
    private JFrame mFrame;         // Holds all the GUI components
    private JDesktopPane mDesktop; // Holds all of the JInternalFrames
    
    private ControlMenu  mainMenu;
    private SensorWindow sensorWindow;
    
    
    
    /**************************************************************************
     * Constructor 
     * 
     * Creates the GUI using the other other windows
     * 
     *************************************************************************/
    
    public WindowManager()
    {
        mFrame = new JFrame(TITLE);
        mDesktop = new JDesktopPane();
        mainMenu = new ControlMenu();
        sensorWindow = new SensorWindow();
        
        
        mDesktop.add(sensorWindow);
        mFrame.add(mDesktop);
        mFrame.setJMenuBar(mainMenu.getInstance());
        
        
        mFrame.setSize(FRAME_W, FRAME_H);
        mFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        mFrame.setLocationRelativeTo(null);
    }
    
    /**************************************************************************
     * quitProgram
     * 
     * 
     *************************************************************************/
    
    public static void quitProgram()
    {
        System.exit(0);
    }
    
    
    /**************************************************************************
     * setVisible
     * 
     * @param isVisible 
     * 
     * 
     *************************************************************************/
    public void setVisible(boolean isVisible)
    {
        mFrame.setVisible(isVisible);
    }
}
