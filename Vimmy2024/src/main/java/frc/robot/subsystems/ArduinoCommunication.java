package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class ArduinoCommunication {
    // this function wraps arduino to port.
    public void wrap() {
        try {
            Constants.arduino = new SerialPort(500000, SerialPort.Port.kUSB);
            System.out.println("Connected on kUSB");
        } catch (Exception e) {
            try {
                Constants.arduino = new SerialPort(500000, SerialPort.Port.kUSB1);
                System.out.println("Connected on kUSB1");
            } catch (Exception e1) {
                try {
                    Constants.arduino = new SerialPort(500000, SerialPort.Port.kUSB2);
                    System.out.println("Connected on kUSB2");
                } catch (Exception e2) {
                    System.out.println("the serial communication with arduino is very confused...");
                }
            }
        }
    }
}
