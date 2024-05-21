package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class ArduinoCommunication {

    //This_function_Finds_arduino_
    
    public static void Wrap() {
        Constants.isArduinoConnected = true;
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
                    Constants.isArduinoConnected = false;
                }
            }
        }
    }
    // this function will recall double value from [adressToPing]
    // uncomment commented code below to have debug messages.
    public static double RecallOneValue(byte adressToPing) {
        if (Constants.isArduinoConnected) {
        double finalValue = -1;
        Constants.arduino.write(new byte[] {adressToPing}, 1);
        if (Constants.arduino.getBytesReceived() > 0) {
            //System.out.println("Handling comminication");
            String readed = Constants.arduino.readString();
            System.out.println("recieved:_"+readed);
            String filtered = "";
            for (int i = 0, p = 0; i < readed.length() ; i++) {
                if (readed.charAt(i) =='.') {
                    p++;
                }
                if (p > 1) {
                    break;
                }
                filtered = filtered + readed.charAt(i);
            }
            //System.out.println("Recieved date succesfull. Data: " + filtered);
            try {
                System.out.println("Filtered: " + filtered);
                finalValue = Double.valueOf(filtered);
            } catch (Exception e) {
                System.out.println(" \u001b[31;1m.The communication is out of sync!!!");
            }
        }
        return finalValue;
        }
        return -3;
    }
}
