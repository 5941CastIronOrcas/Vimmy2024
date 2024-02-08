package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class ArduinoCommunication {
    // this function wraps arduino to port.
    public static void Wrap() {
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
    // this function will recall double value from [adressToPing]
    // uncomment commented code below to have debug messages.
    public static double RecallOneValue(byte adressToPing) {
        double finalValue = -1;
        Constants.arduino.write(new byte[] {adressToPing}, 1);
        if (Constants.arduino.getBytesReceived() > 0) {
            //System.out.println("Handling comminication");
            String readed = Constants.arduino.readString();
            //System.out.println("Recieved date succesfull. Data: " + readed);
            try {
                finalValue = Double.valueOf(readed);
            } catch (Exception e) {
                System.out.println(" \u001b[31;1m.The communication is out of sync!!!");
            }
        }
        return finalValue;
    }
}
