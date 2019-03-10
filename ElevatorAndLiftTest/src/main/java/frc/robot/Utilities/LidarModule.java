/**----------------------------------------------------------------------------*/
/** Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/** Open Source Software - may be modified and shared by FRC teams. The code   */
/** must be accompanied by the FIRST BSD license file in the root directory of */
/** the project.                                                               */
/**----------------------------------------------------------------------------*/

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * Class to create a LidarModule object, allowing for object oriented communication with
 * The TFMini LIDAR module.
 * TODO: Test this class.
 */
public class LidarModule
{

    // Private consants to help with communication
    private final int TFMINI_BAUDRATE = 115200;
    private final int TFMINI_FRAME_SIZE = 7;
    private final int TFMINI_MAXBYTESBEFOREHEADER = 30;
    private final int TFMINI_MAX_MEASUREMENT_ATTEMPTS = 10;

    private final int READY = 0;
    private final int ERROR_SERIAL_NOHEADER = 1;
    private final int ERROR_SERIAL_BADCHECKSUM = 2;
    private final int ERROR_SERIAL_TOOMANYTRIES = 3;
    private final int MEASUREMENT_OK = 10;

    private SerialPort serialPort = null; // SerialPort object required for communication with TFMini

    private int state, distance, strength; // Treat these as unsigned integers

    public LidarModule(Port port)
    {
        serialPort = new SerialPort(TFMINI_BAUDRATE, port);
        strength = -1;
        distance = -1;
        state = READY;

        setStandardOutputMode();
    }

    /**
    * Return the distance that the sensor reads, or error out
    */
    public int getDistance()
    {
        int numMeasurementAttempts = 0;
        while(takeMeasurement() != 0)
        {
            numMeasurementAttempts++;
            if (numMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS)
            {
                System.out.println("TF Mini error: too many measurement attempts");
                System.out.println("Last error:");
                if (state == ERROR_SERIAL_NOHEADER)     System.out.println("ERROR_SERIAL_NOHEADER");
                if (state == ERROR_SERIAL_BADCHECKSUM)  System.out.println("ERROR_SERIAL_BADCHECKSUM");
                if (state == ERROR_SERIAL_TOOMANYTRIES) System.out.println("ERROR_SERIAL_TOOMANYTRIES");
                
                state = ERROR_SERIAL_TOOMANYTRIES;
                distance = -1;
                strength = -1;
                return -1;
            }
        }

        if (state == MEASUREMENT_OK)
        {
            return distance;
        }
        return -1;
    }

    /**
    * Handles the low-level communication with the TFMini
    */
    public int takeMeasurement()
    {
        int numBytesRead = 0;
        byte lastByte = 0x00;

        /**
        * Read the serial stream until we see the beginning of the TF Mini header,
        * or we timeout reading too many characters.
        */
        while(true)
        {
            if (serialPort.getBytesReceived() > 0)
            {
                byte currentByte = serialPort.read(1)[0];

                if (lastByte == 0x59 && currentByte == 0x59)
                {
                    break; // Break to begin the frame
                }
                else
                {
                    // If we haven't seen two 0x59's in a row, keep reading.
                    lastByte = currentByte;
                    numBytesRead++;
                }
            }

            /**
            * If we read more than 'x' characters without finding a frame header
            * then it's likely there is an issue with the Serial connection,
            * and we should timeout and throw an error.
            */
            if (numBytesRead > TFMINI_MAXBYTESBEFOREHEADER)
            {
                state = ERROR_SERIAL_NOHEADER;
                distance = -1;
                strength = -1;
                return -1;
            }
        }

        byte[] frame = new byte[TFMINI_FRAME_SIZE];

        byte checksum = (byte)(0x59 + 0x59);

        for (int i = 0; i < TFMINI_FRAME_SIZE; i++)
        {
            // Wait for the serial port to recieve data
            while(serialPort.getBytesReceived() < 1)
            {
                // Do nothing
            }

            // Read one byte
            frame[i] = serialPort.read(1)[0];

            // Store running checksum
            if (i < TFMINI_FRAME_SIZE - 2)
            {
                checksum += frame[i];
            }
        }

        /**
        * The last byte in the frame is a checksum, so we can compare it to ours.
        */
        byte checksumByte = frame[TFMINI_FRAME_SIZE - 1];
        if (checksum != checksumByte)
        {
            state = ERROR_SERIAL_BADCHECKSUM;
            distance = -1;
            strength = -1;
            return -1;
        }

        /**
        * Interpret the recieved data.
        */
        int dist = (frame[1] << 8) + frame[0];
        int st = (frame[3] << 8) + frame[2];

        /**
        * Store the values
        */
        distance = dist;
        strength = st;
        state = MEASUREMENT_OK;

        return 0; // 0 for success
    }

    /**
    * Returns the current strength reading from TFMini
    */
    public int getRecentSignalStrength()
    {
        return strength;
    }

    /**
    * Sets the TFMini to "standard" output mode
    */
    public void setStandardOutputMode()
    {
        byte[] buffer = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06};
        serialPort.write(buffer, buffer.length);
        serialPort.flush();
    }

    /**
    * Sets the TFMini to advanced parameter configuration mode
    */
    public void setConfigMode()
    {
        byte[] buffer = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x02};
        serialPort.write(buffer, buffer.length);
        serialPort.flush();
    }

    /**
    * Sets the TFMini to single scan external trigger mode
    */
    public void setSingleScanMode()
    {
        setConfigMode();
        byte[] buffer = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x00, 0x40};
        serialPort.write(buffer, buffer.length);
        serialPort.flush();
    }

    /**
    * Sends an external trigger to the TFMini
    */
    public void externalTrigger()
    {
        setConfigMode();
        byte[] buffer = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x00, 0x41};
        serialPort.write(buffer, buffer.length);
        serialPort.flush();
    }

}
