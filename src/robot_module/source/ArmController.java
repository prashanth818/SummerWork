import processing.core.*;
import processing.serial.*;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.HashMap;
import java.util.ArrayList;
import java.io.File;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;
import py4j.GatewayServer;

public class ArmController extends PApplet{
  Serial sPort;               //serial port object, used to connect to a serial port and send data to the ArbotiX

  int numSerialPorts = Serial.list().length;                 //Number of serial ports available at startup
  String[] serialPortString = new String[numSerialPorts+1];  //string array to the name of each serial port - used for populating the drop down menu
  int selectedSerialPort;                                    //currently selected port from serialList drop down

  boolean updateFlag = false;     //trip flag, true when the program needs to send a serial packet at the next interval, used by both 'update' and 'autoUpdate' controls
  int updatePeriod = 33;          //minimum period between packet in Milliseconds , 33ms = 30Hz which is the standard for the commander/arm link protocol

  long prevCommandTime = 0;       //timestamp for the last time that the program sent a serial packet
  long heartbeatTime = 0;         //timestamp for the last time that the program received a serial packet from the Arm
  long currentTime = 0;           //timestamp for currrent time

  int packetRepsonseTimeout = 5000;      //time to wait for a response from the ArbotiX Robocontroller / Arm Link Protocol

  int currentArm = 0;          //ID of current arm. 1 = pincher, 2 = reactor, 3 = widowX, 5 = snapper
  int currentMode = 0;         //Current IK mode, 1=Cartesian, 2 = cylindrical, 3= backhoe
  int currentOrientation = 0;  //Current wrist oritnation 1 = straight/normal, 2=90 degrees

  //booleans for key tracking
  boolean xkey = false;
  boolean ykey = false;
  boolean zkey = false;
  boolean wangkey = false;
  boolean wrotkey = false;
  boolean gkey = false;
  boolean dkey = false;

  int startupWaitTime = 10000;    //time in ms for the program to wait for a response from the ArbotiX
  Serial[] sPorts = new Serial[numSerialPorts];  //array of serial ports, one for each avaialable serial port.

  int armPortIndex = -1; //the index of the serial port that an arm is currently connected to(relative to the list of avaialble serial ports). -1 = no arm connected

  int analogSampleTime = 33;//time between analog samples
  long lastAnalogSample = millis();//
  int nextAnalog = 0;
  int[]analogValues = new int[8];

  int currentPose = 0;  //current pose that has been selected.


  CopyOnWriteArrayList<int[]> poseData;
  int[] blankPose = new int[9]; //blank pose : x, y, z, wristangle, wristRotate, Gripper, Delta, digitals, pause


  int[] defaultPose = {
    0, 200, 200, 0, 0, 256, 125, 0, 1000
  }; //blank pose : x, y, z, wristangle, wristRotate, Gripper, Delta, digitals

  boolean playSequence = false;
  //boolean waitForResponse = false;
  int lastTime;
  int lastPose;



  int connectFlag = 0;
  int disconnectFlag = 0;
  int autoConnectFlag = 0;

  int pauseTime = 1000;
  public void pythonTest(){
    System.out.println("I got here");
  }

  public static void main(String[] args) {
    GatewayServer gatewayServer = new GatewayServer(new ArmController());
    gatewayServer.start();
    System.out.println("Gateway Server Started");
  }

  public boolean connectArm(){
    boolean success;
    for (int i=Serial.list().length-1; i>=0; i--)
    {
      System.out.println("port"+i);
      //try to connect to the port at 38400bps, otherwise show an error message
      try
      {
        System.out.println("serial being made");
        sPorts[i] = new Serial(this, Serial.list()[i], 38400);
        System.out.println("serial made");
      }
      catch(Exception e)
      {
        System.out.println("Error Opening Serial Port "+Serial.list()[i] + " for auto search");
        sPorts[i] = null;
      }
    }
    if (checkArmStartup() == true)
    {
      System.out.println("Arm Found from auto search on port "+Serial.list()[armPortIndex]);
      System.out.println("Connected");
      success = true;
    }else{
      System.out.println("Arm not found");
      success = false;
    }

    for (int i=0; i<numSerialPorts; i++)
    {
      //if the index being scanned is not the index of an port with an arm connected, stop/null the port
      //if the port is already null, then it was never opened
      if (armPortIndex != i & sPorts[i] != null)
      {
        System.out.println("Stopping port "+Serial.list()[i]) ;
        sPorts[i].stop();
        sPorts[i] = null;
      }
    }
    return success;
  }



  public void disconnectArm(){
    sPorts[armPortIndex].stop();
    sPorts[armPortIndex] = null;
    currentMode = 0;
    currentArm = 0;
    currentOrientation = 0;
  }


  public boolean verifyPacket(byte[] returnPacket)
  {
    int packetLength = returnPacket.length;  //length of the packet
    int tempChecksum = 0; //int for temporary checksum calculation
    byte localChecksum; //local checksum calculated by processing

    //check header, which should always be 255/0xff
    if(returnPacket[0] == PApplet.parseByte(255))
    {
        //iterate through bytes # 1 through packetLength-1 (do not include header(0) or checksum(packetLength)
        for(int i = 1; i<packetLength-1;i++)
        {
          tempChecksum = (int) returnPacket[i] + tempChecksum;//add byte value to checksum
        }

        localChecksum = PApplet.parseByte(~(tempChecksum % 256)); //calculate checksum locally - modulus 256 to islotate bottom byte, then invert(~)

        //check if calculated checksum matches the one in the packet
        if(localChecksum == returnPacket[packetLength-1])
        {
          //check is the error packet is 0, which indicates no error
          if(returnPacket[3] == 0)
          {
            //check that the arm id packet is a valid arm
            if(returnPacket[1] == 1 || returnPacket[1] == 2 || returnPacket[1] == 3 || returnPacket[1] == 5)
            {
              System.out.println("verifyPacket Success!");
              return(true);
            }
            else {System.out.println("verifyPacket Error: Invalid Arm Detected! Arm ID:"+returnPacket[1]);}
          }
          else {System.out.println("verifyPacket Error: Error Packet Reports:"+returnPacket[3]);}
        }
        else {System.out.println("verifyPacket Error: Checksum does not match: Returned:"+ returnPacket[packetLength-1] +" Calculated:"+localChecksum );}
    }
    else {System.out.println("verifyPacket Error: No Header!");}
    return(false);

  }

  public boolean checkArmStartup()
  {
    byte[] returnPacket = new byte[5];  //byte array to hold return packet, which is 5 bytes long
    long startTime = millis();
    long currentTime = startTime;
    // System.out.println("Checking for arm on startup ");
    while(currentTime - startTime < startupWaitTime )
    {
      delayMs(100);  //The ArbotiX has a delay of 50ms between starting the serial continueing the program, include an extra 10ms for other ArbotiX startup tasks
      for(int i = 0; i< sPorts.length;i++)
      {
        if(sPorts[i] != null)
        {
          armPortIndex = i;

          // System.out.println("Checking for arm on startup - index# " + i);
          sendCommanderPacket(0, 200, 200, 0, 512, 256, 128, 0, 112);    //send a commander style packet - the first 8 bytes are inconsequntial, only the last byte matters. '112' is the extended byte that will request an ID packet
          returnPacket = readFromArm(5,false);//read raw data from arm, complete with wait time

          if(verifyPacket(returnPacket) == true)
          {
            currentArm = returnPacket[1]; //set the current arm based on the return packet
            // System.out.println("Startup Arm #" +currentArm+ " Found");
//            setPositionParameters();      //set the GUI default/min/maxes and field lables

            return(true) ;                //Return a true signal to signal that an arm has been found
          }
        }
      }

      currentTime = millis();
    }
    armPortIndex = -1;
    return(false);


  }

  public boolean moveArmToHome()
  {
    System.out.println("Attempting to put arm in Home Mode - ");
    sendCommanderPacket(0,0,0,0,0,0,0,0,32);//only the last/extended byte matters - 96 signals the arm to go to sleep

    byte[] returnPacket = new byte[5];//return id packet is 5 bytes long
    System.out.println("Packet sent");
    returnPacket = readFromArm(5,true);//read raw data from arm
    if(verifyPacket(returnPacket) == true)
    {
      // printlnDebug("Sleep mode success!");
      System.out.println("Home mode success");
      return(true) ;
    }
    else
    {
      //printlnDebug("Sleep mode-No return packet detected");
      //displayError("There was a problem putting the arm in sleep mode","");
      return(false);
    }
  }

  public boolean putArmToSleep()
  {
    System.out.println("Attempting to put arm in sleep mode - ");
    sendCommanderPacket(100,200,0,0,0,0,0,0,96);//only the last/extended byte matters - 96 signals the arm to go to sleep

    byte[] returnPacket = new byte[5];//return id packet is 5 bytes long
    returnPacket = readFromArm(5,true);//read raw data from arm
    if(verifyPacket(returnPacket) == true)
    {
      //printlnDebug("Sleep mode success!");
      return(true) ;
    }
    else
    {
      //printlnDebug("Sleep mode-No return packet detected");
      //displayError("There was a problem putting the arm in sleep mode","");
      return(false);
    }
  }
   public byte[] intToBytes(int convertInt)
   {
     byte[] returnBytes = new byte[2]; // array that holds the two bytes to return
     byte mask = PApplet.parseByte(255);          //mask for the low byte (255/0xff)
     returnBytes[0] = PApplet.parseByte(convertInt & mask);//low byte - perform an '&' operation with the byte mask to remove the high byte
     returnBytes[1] = PApplet.parseByte((convertInt>>8) & mask);//high byte - shift the byte to the right 8 bits. perform an '&' operation with the byte mask to remove any additional data
     return(returnBytes);  //return byte array

   }

  public void sendCommanderPacket(int x, int y, int z, int wristAngle, int wristRotate, int gripper, int delta, int button, int extended)
  {
    System.out.println(armPortIndex);
    sPorts[armPortIndex].clear();//clear the serial port for the next round of communications

    //convert each positional integer into 2 bytes using intToBytes()
    byte[] xValBytes = intToBytes(x);
    byte[] yValBytes = intToBytes(y);
    byte[] zValBytes =  intToBytes(z);
    byte[] wristAngleValBytes = intToBytes(wristAngle);
    byte[] wristRotValBytes = intToBytes(wristRotate);
    byte[] gripperValBytes = intToBytes(gripper);
    //cast int to bytes
    byte buttonByte = PApplet.parseByte(button);
    byte extValByte = PApplet.parseByte(extended);
    byte deltaValByte = PApplet.parseByte(delta);
    boolean flag = true;
    //calculate checksum - add all values, take lower byte (%256) and invert result (~). you can also invert results by (255-sum)
    byte checksum = PApplet.parseByte(~(xValBytes[1]+xValBytes[0]+yValBytes[1]+yValBytes[0]+zValBytes[1]+zValBytes[0]+wristAngleValBytes[1]+wristAngleValBytes[0]+wristRotValBytes[1]+wristRotValBytes[0]+gripperValBytes[1]+gripperValBytes[0]+deltaValByte + buttonByte+extValByte)%256);
    System.out.println("All bytes");
    //send commander style packet. Following labels are for cartesian mode, see function comments for clyindrical/backhoe mode
      //try to write the first header byte
      try
      {
        sPorts[armPortIndex].write(0xff);//header
      }
      //catch an exception in case of serial port problems
      catch(Exception e)
      {
         flag = false;
      }
      if(flag == true)
      {
        sPorts[armPortIndex].write(xValBytes[1]); //X Coord High Byte
        sPorts[armPortIndex].write(xValBytes[0]); //X Coord Low Byte
        sPorts[armPortIndex].write(yValBytes[1]); //Y Coord High Byte
        sPorts[armPortIndex].write(yValBytes[0]); //Y Coord Low Byte
        sPorts[armPortIndex].write(zValBytes[1]); //Z Coord High Byte
        sPorts[armPortIndex].write(zValBytes[0]); //Z Coord Low Byte
        sPorts[armPortIndex].write(wristAngleValBytes[1]); //Wrist Angle High Byte
        sPorts[armPortIndex].write(wristAngleValBytes[0]); //Wrist Angle Low Byte
        sPorts[armPortIndex].write(wristRotValBytes[1]); //Wrist Rotate High Byte
        sPorts[armPortIndex].write(wristRotValBytes[0]); //Wrist Rotate Low Byte
        sPorts[armPortIndex].write(gripperValBytes[1]); //Gripper High Byte
        sPorts[armPortIndex].write(gripperValBytes[0]); //Gripper Low Byte
        sPorts[armPortIndex].write(deltaValByte); //Delta Low Byte
        sPorts[armPortIndex].write(buttonByte); //Button byte
        sPorts[armPortIndex].write(extValByte); //Extended instruction
        sPorts[armPortIndex].write(checksum);  //checksum
      }
  }

  public byte[] readFromArm(int bytesExpected, boolean wait)
  {
    byte[] responseBytes = new byte[bytesExpected];    //byte array to hold response data
    delayMs(100);//wait a minimum 100ms to ensure that the controller has responded - this applies to both wait==true and wait==false conditions

    byte bufferByte = 0;  //current byte that is being read
    long startReadingTime = millis();//time that the program started looking for data

    System.out.println("Incoming Raw Packet from readFromArm():"); //debug

    //if the 'wait' flag is TRUE this loop will wait until the serial port has data OR it has waited more than packetRepsonseTimeout milliseconds.
    //packetRepsonseTimeout is a global variable

    while(wait == true & sPorts[armPortIndex].available() < bytesExpected  & millis()-startReadingTime < packetRepsonseTimeout)
    {
       //do nothing, just waiting for a response or timeout
    }

    for(int i =0; i < bytesExpected;i++)
    {
      // If data is available in the serial port, continute
      if(sPorts[armPortIndex].available() > 0)
      {
        bufferByte = PApplet.parseByte(sPorts[armPortIndex].readChar());
        responseBytes[i] = bufferByte;
        System.out.println(hex(bufferByte) + "-"); //debug
      }
      else
      {
        System.out.println("NO BYTE-");//debug
      }
    }//end looking for bytes from packet
    System.out.println(" "); //debug  finish line

    sPorts[armPortIndex].clear();  //clear serial port for the next read

    return(responseBytes);  //return serial data
  }

  public void delayMs(int ms)
  {

    int time = millis();  //time that the program starts the loop
    while(millis()-time < ms)
    {
       //loop/do nothing until the different between the current time and 'time'
    }
  }

}
