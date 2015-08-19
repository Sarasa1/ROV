/******************************************************************************
 * ControlMenu.java
 * 
 * Creates the menu that will be added to the ROV GUI
 * 
 * 
 *****************************************************************************/


package rov.gui.window;

import rov.gui.*;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;

// JERRY'S NOTE: THE CLASS IS FINAL SO THAT IT CAN'T BE CHANGED AND THE
// METHODS CAN'T BE OVERWRITTEN
public final class ControlMenu extends JMenuBar implements ActionListener
{
    private JMenuBar menuBar;
    private JMenu menu;
    
    /**************************************************************************
     * ControlMenu 
     * 
     * Creates the menu bar for the ROV GUI by calling createMenu()
     * 
     *************************************************************************/
    
    public ControlMenu()
    {
        menuBar = new JMenuBar();
        createMenu();
    }
    
    /**************************************************************************
     * createMenu
     * 
     * 
     * 
     *************************************************************************/
    protected void createMenu()
    {
        // Create the "File" menu item
        menu = new JMenu("File");
        menu.setMnemonic(KeyEvent.VK_F); // Set shortcut to alt + F
        
        // Add the "New" Menu item
        // "New" is a submenu that has options to create new items, or to
        // recreate all of the items
        // Shortcut to open the "New Menu" is Alt + N
        JMenu newMenu = new JMenu("New");
        newMenu.setMnemonic(KeyEvent.VK_N);
        
        // Add option to create a new set of windows
        // Shortcut is Alt + W
        JMenuItem newWindows = new JMenuItem("Windows");
        newWindows.setMnemonic(KeyEvent.VK_W);
        newWindows.setAccelerator(KeyStroke.getKeyStroke( 
                                  KeyEvent.VK_W, ActionEvent.ALT_MASK));
        newWindows.setActionCommand("new windows");
        newWindows.addActionListener(this);
        newMenu.add(newWindows);
        
        // Add option to create new Sensor Window
        // Shortcut is Alt + S
        JMenuItem newSensor = new JMenuItem("Sensor Window");
        newSensor.setMnemonic(KeyEvent.VK_S);
        newSensor.setAccelerator(KeyStroke.getKeyStroke(
                                 KeyEvent.VK_S, ActionEvent.ALT_MASK));
        newSensor.setActionCommand("new sensor");
        newSensor.addActionListener(this);
        newMenu.add(newSensor);
        
        // Add option to create new Camera Stream Window
        // Shortcut is Alt + C
        JMenuItem newCamera = new JMenuItem("Camera Window");
        newCamera.setMnemonic(KeyEvent.VK_C);
        newCamera.setAccelerator(KeyStroke.getKeyStroke(
                                 KeyEvent.VK_C, ActionEvent.ALT_MASK));
        newCamera.setActionCommand("new camera");
        newCamera.addActionListener(this);
        newMenu.add(newCamera);
        
        // Add the sub menu to the menu
        menu.add(newMenu);
        
        // Add option to exit the GUI
        // Shortcut is Alt + Q
        JMenuItem quitItem = new JMenuItem("Quit");
        quitItem.setMnemonic(KeyEvent.VK_Q);
        quitItem.setAccelerator(KeyStroke.getKeyStroke(
                                KeyEvent.VK_Q, ActionEvent.ALT_MASK));
        quitItem.setActionCommand("quit");
        quitItem.addActionListener(this);
        menu.add(quitItem);
        
        // Add the menu to the menu bar
        menuBar.add(menu);
    }
    
    /**************************************************************************
     * actionPerformed
     * 
     * @param e 
     * 
     *************************************************************************/
    @Override
    public void actionPerformed(ActionEvent e)
    {
        // Check what menuItem was pressed
        if (e.getActionCommand() != null) { 
            switch (e.getActionCommand()) {
                case "new windows":
                    System.out.println("Create New Windows\n");
                    break;
                    
                case "new sensor":
                    System.out.println("Create a New Sensor Window\n");
                    break;
                    
                case "new camera":
                    System.out.println("Create a New Camera Window\n");
                    break;
                    
                case "quit":
                    System.out.println("Quitting Program\n");
                    WindowManager.quitProgram();
                    break;
            }
        }
    }
    
    /**************************************************************************
     * getInstance()
     * 
     * Gets an instance of the ControlMenu class
     * 
     * @return an instance of Control Menu
     * 
     *************************************************************************/
    public JMenuBar getInstance()
    {
        return menuBar;
    }
    
    /**************************************************************************
     * getMenu
     *
     * Gets an instance of the Menu used for use in other places
     * 
     * @return a menu
     * 
     *************************************************************************/
    
    public JMenu getMenu()
    {
        return menu;
    }
}
