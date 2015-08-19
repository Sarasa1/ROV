package rov.gui;

import javax.swing.SwingUtilities;
import rov.gui.demos.*;

public class Main {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                
                // Create the controller window and make it visible
                WindowManager wm = new WindowManager();
                //InternalFrameDemo.createAndShowGUI();
                wm.setVisible(true);
            }
        });
    }
}
