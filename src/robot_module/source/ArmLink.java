import serial.*;
import java.awt.Font;
import java.util.concurrent.CopyOnWriteArrayList;

import java.util.HashMap;
import java.util.ArrayList;
import java.io.File;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;

public class ArmLink {
Serial sPort;               //serial port object, used to connect to a serial port and send data to the ArbotiX

PrintWriter debugOutput;        //output object to write to a file

int numSerialPorts = Serial.list().length;                 //Number of serial ports available at startup
String[] serialPortString = new String[numSerialPorts+1];  //string array to the name of each serial port - used for populating the drop down menu
int selectedSerialPort;                                    //currently selected port from serialList drop down

boolean debugConsole = true;      //change to 'false' to disable debuging messages to the console, 'true' to enable
boolean debugFile = false;        //change to 'false' to disable debuging messages to a file, 'true' to enable

boolean debugGuiEvent = true;     //change to 'false' to disable GUI debuging messages, 'true' to enable
boolean debugSerialEvent = false;     //change to 'false' to disable GUI debuging messages, 'true' to enable
//int lf = 10;    // Linefeed in ASCII

boolean debugFileCreated  = false;  //flag to see if the debug file has been created yet or not

boolean enableAnalog = false; //flag to enable reading analog inputs from the Arbotix

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


/********DRAG AND DROP VARS*/
int numPanels =0;
int currentTopPanel = 0;
int dragFlag = -1;
int panelsX = 100;  //x coordinate for all panels
int panelsYStart = 25;//distance between top of parent and first panel
int panelYOffset = 25;//distance between panels
int panelHeight = 15;//height of the panel
int lastDraggedOverId = -1;
int lastDraggedOverColor = -1;
int numberPanelsDisplay = 14;
int draggingPosition = -1;
float draggingY = 0;

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
int cameraFlag = 2;


Capture cam;


int pauseTime = 1000;

/***********/

public void setup() {
  size(475, 733, JAVA2D);  //draw initial screen //475
  poseData = new CopyOnWriteArrayList<int[]>();

  createGUI();   //draw GUI components defined in gui.pde

  //Build Serial Port List
  serialPortString[0] = "Serial Port";   //first item in the list will be "Serial Port" to act as a label
  //iterate through each avaialable serial port
  for (int i=0; i<numSerialPorts; i++)
  {
    serialPortString[i+1] = Serial.list()[i];  //add the current serial port to the list, add one to the index to account for the first item/label "Serial Port"
  }
  serialList.setItems(serialPortString, 0);  //add contents of srialPortString[] to the serialList GUI

  prepareExitHandler();//exit handler for clearing/stopping file handler
}

//Main Loop
public void draw()
{

  background(128);//draw background color
  image(logoImg, 5, 0, 280, 50);  //draw logo image
  image(footerImg, 15, 770);      //draw footer image

  currentTime = millis();  //get current timestamp

  if (connectFlag ==1)
  {
    if (selectedSerialPort > -1)
    {
      try
      {
        sPorts[selectedSerialPort] =  new Serial(this, Serial.list()[selectedSerialPort], 38400);
      }
      catch(Exception e)
      {
        printlnDebug("Error Opening Serial Port"+serialList.getSelectedText());
        sPorts[selectedSerialPort] = null;
        displayError("Unable to open selected serial port" + serialList.getSelectedText() +". See link for possible solutions.", "http://learn.trossenrobotics.com/arbotix/8-advanced-used-of-the-tr-dynamixel-servo-tool");
      }
    }

    //check to see if the serial port connection has been made
    if (sPorts[selectedSerialPort] != null)
    {

      //try to communicate with arm
      if (checkArmStartup() == true)
      {
        // wait for user to input data to move the arm.
        //disable connect button and serial list
        serialList.setEnabled(false);
        serialList.setAlpha(128);
        autoConnectButton.setEnabled(false);
        autoConnectButton.setAlpha(128);
        //enable disconnect button
        disconnectButton.setEnabled(true);
        disconnectButton.setAlpha(255);

        //enable & set visible control and mode panel
        modePanel.setVisible(true);
        modePanel.setEnabled(true);
        controlPanel.setVisible(true);
        controlPanel.setEnabled(true);
        sequencePanel.setVisible(true);
        sequencePanel.setEnabled(true);
        ioPanel.setVisible(true);
        ioPanel.setEnabled(true);
        delayMs(100);//short delay
        setCartesian();
        statusLabel.setText("Connected");
      }

      //if arm is not found return an error
      else
      {
        sPorts[selectedSerialPort].stop();
        //      sPorts.get(selectedSerialPort) = null;
        sPorts[selectedSerialPort] = null;
        printlnDebug("No Arm Found on port "+serialList.getSelectedText()) ;

        displayError("No Arm found on serial port" + serialList.getSelectedText() +". Make sure power is on and the arm is connected to the computer.", "http://learn.trossenrobotics.com/arbotix/8-advanced-used-of-the-tr-dynamixel-servo-tool");

        statusLabel.setText("Not Connected");
      }
    }

    connectFlag = 0;
  }


  if (disconnectFlag == 1)
  {

    autoUpdateCheckbox.setSelected(false);
    putArmToSleep();
    //TODO: call response & check

    ///stop/disconnect the serial port and set sPort to null for future checks
    sPorts[armPortIndex].stop();
    sPorts[armPortIndex] = null;

    //enable connect button and serial port
    connectButton.setEnabled(true);
    connectButton.setAlpha(255);
    serialList.setEnabled(true);
    serialList.setAlpha(255);
    autoConnectButton.setEnabled(true);
    autoConnectButton.setAlpha(255);

    //disable disconnect button
    disconnectButton.setEnabled(false);
    disconnectButton.setAlpha(128);
    //disable & set invisible control and mode panel
    controlPanel.setVisible(false);
    controlPanel.setEnabled(false);
    sequencePanel.setVisible(false);
    sequencePanel.setEnabled(false);
    modePanel.setVisible(false);
    modePanel.setEnabled(false);
    ioPanel.setVisible(false);
    ioPanel.setEnabled(false);
    wristPanel.setVisible(false);
    wristPanel.setEnabled(false);

    //uncheck all checkboxes to reset
    autoUpdateCheckbox.setSelected(false);
    //digitalCheckbox0.setSelected(false);
    digitalCheckbox1.setSelected(false);
    digitalCheckbox2.setSelected(false);
    digitalCheckbox3.setSelected(false);
    digitalCheckbox4.setSelected(false);
    digitalCheckbox5.setSelected(false);
    digitalCheckbox6.setSelected(false);
    digitalCheckbox7.setSelected(false);

    analogCheckbox.setSelected(false);

    //set arm/mode/orientation to default
    currentMode = 0;
    currentArm = 0;
    currentOrientation = 0;

    //reset button color mode
    cartesianModeButton.setLocalColorScheme(GCScheme.CYAN_SCHEME);
    cylindricalModeButton.setLocalColorScheme(GCScheme.CYAN_SCHEME);
    backhoeModeButton.setLocalColorScheme(GCScheme.CYAN_SCHEME);

    //reset alpha trapsnparency on orientation buttons
    //DEPRECATED armStraightButton.setAlpha(128);
    //DEPRECATEDarm90Button.setAlpha(128);


    disconnectFlag = 0;
    statusLabel.setText("Not Connected");
  }



  if (autoConnectFlag == 1)
  {


    //disable connect button and serial list
    connectButton.setEnabled(false);
    connectButton.setAlpha(128);
    serialList.setEnabled(false);
    serialList.setAlpha(128);
    autoConnectButton.setEnabled(false);
    autoConnectButton.setAlpha(128);
    //enable disconnect button
    disconnectButton.setEnabled(true);
    disconnectButton.setAlpha(255);

    //for (int i=0;i<Serial.list().length;i++) //scan from bottom to top
    //scan from the top of the list to the bottom, for most users the ArbotiX will be the most recently added ftdi device
    for (int i=Serial.list ().length-1; i>=0; i--)
    {
      println("port"+i);
      //try to connect to the port at 38400bps, otherwise show an error message
      try
      {
        sPorts[i] = new Serial(this, Serial.list()[i], 38400);
      }
      catch(Exception e)
      {
        printlnDebug("Error Opening Serial Port "+Serial.list()[i] + " for auto search");
        sPorts[i] = null;
      }
    }//end interating through serial list

    //try to communicate with arm
    if (checkArmStartup() == true)
    {
      printlnDebug("Arm Found from auto search on port "+Serial.list()[armPortIndex]) ;

      //enable & set visible control and mode panel, enable disconnect button
      modePanel.setVisible(true);
      modePanel.setEnabled(true);
      controlPanel.setVisible(true);
      controlPanel.setEnabled(true);
      sequencePanel.setVisible(true);
      sequencePanel.setEnabled(true);
      ioPanel.setVisible(true);
      ioPanel.setEnabled(true);
      disconnectButton.setEnabled(true);
      delayMs(200);//shot delay
      setCartesian();

      statusLabel.setText("Connected");

      //break;
    }

    //if arm is not found return an error
    else
    {
      //enable connect button and serial port
      connectButton.setEnabled(true);
      connectButton.setAlpha(255);
      serialList.setEnabled(true);
      serialList.setAlpha(255);
      autoConnectButton.setEnabled(true);
      autoConnectButton.setAlpha(255);

      //disable disconnect button
      disconnectButton.setEnabled(false);
      disconnectButton.setAlpha(128);
      //disable & set invisible control and mode panel

      displayError("No Arm found using auto seach. Please check power and connections", "");
      statusLabel.setText("Not Connected");
    }
    //stop all serial ports without an arm connected
    for (int i=0; i<numSerialPorts; i++)
    {
      //if the index being scanned is not the index of an port with an arm connected, stop/null the port
      //if the port is already null, then it was never opened
      if (armPortIndex != i & sPorts[i] != null)
      {
        printlnDebug("Stopping port "+Serial.list()[i]) ;
        sPorts[i].stop();
        sPorts[i] = null;
      }
    }


    autoConnectFlag = 0;
  }




  //check if
  //  -update flag is true, and a packet needs to be sent
  //  --it has been more than 'updatePeriod' ms since the last packet was sent
  if (currentTime - prevCommandTime > updatePeriod )
  {



    //check if
    //--analog retrieval is enabled
    //it has been long enough since the last sample
    if (currentTime - lastAnalogSample > analogSampleTime && (true == enableAnalog))
    {
      if ( currentArm != 0)
      {
        println("analog");

        analogValues[nextAnalog] = analogRead(nextAnalog);
        analogLabel[nextAnalog].setText(
        Integer.toString(nextAnalog) + ":" + Integer.toString(analogValues[nextAnalog]));

        nextAnalog = nextAnalog+1;
        if (nextAnalog > 7)
        {
          nextAnalog = 0;
          lastAnalogSample = millis();
        }
      }
    }






    //check if
    //  -update flag is true, and a packet needs to be sent
    else if (updateFlag == true)
    {
      updateOffsetCoordinates();     //prepare the currentOffset coordinates for the program to send
      updateButtonByte();  //conver the current 'digital button' checkboxes into a value to be sent to the arbotix/arm
      prevCommandTime = currentTime; //update the prevCommandTime timestamp , used to calulcate the time the program can next send a command


      //check that the serial port is active - if the 'armPortIndex' variable is not -1, then a port has been connected and has an arm attached
      if (armPortIndex > -1)
      {
        //send commander packet with the current global currentOffset coordinatges
        sendCommanderPacket(xCurrentOffset, yCurrentOffset, zCurrentOffset, wristAngleCurrentOffset, wristRotateCurrentOffset, gripperCurrentOffset, deltaCurrentOffset, digitalButtonByte, extendedByte);

        //if a sequence is playing, wait for the arm response before moving on
        //        if(playSequence ==true)
        //        {
        //          //use this code to enable return packet checking for positional commands
        //          byte[] responseBytes = new byte[5];    //byte array to hold response data
        //          //responseBytes = readFromArm(5);//read raw data from arm, complete with wait time
        //          responseBytes = readFromArmFast(5);
        //        }
        //        if(verifyPacket(responseBytes) == true)
        //        {
        //          printlnDebug("Moved!");
        //        }
        //        else
        //        {
        //          printlnDebug("No Arm Found");
        //        }
      }

      //in normal update mode, pressing the update button signals the program to send a packet. In this
      //case the program must set the update flag to false in order to stop new packets from being sent
      //until the update button is pressed again.
      //However in autoUpdate mode, the program should not change this flag (only unchecking the auto update flag should set the flag to false)
      if (autoUpdateCheckbox.isSelected() == false)
      {
        updateFlag = false;//only set the updateFlag to false if the autoUpdate flag is false
      }
      //use this oppurtunity to set the extended byte to 0 if autoupdate is enabled - this way the extended packet only gets sent once
      else
      {
        if (extendedByte != 0)
        {
          extendedByte = 0;
          extendedTextField.setText("0");
        }
      }
    }//end command code
  }



  //DRAG AND DROP CODE
  //check if the 'dragFlag' is set
  if (dragFlag > -1)
  {

    int dragPanelNumber = dragFlag - currentTopPanel;  //dragPanelNumber now has the panel # (of the panel that was just dragged) relative to the panels that are currently being displayed.

    float dragPanelY = poses.get(dragFlag).getY();  //the final y coordinate of the panel that was last dragged

    int newPanelPlacement = floor((dragPanelY - panelsYStart)/25);//determine the panel #(relative to panels being shown) that the dragged panel should displace

    //set bounds for dragging panels too high/low
    newPanelPlacement = max(0, newPanelPlacement);//for negative numbers (i.e. dragged above first panel) set new panel to '0'
    newPanelPlacement = min(min(numberPanelsDisplay-1, poses.size())-1, newPanelPlacement);//for numbers that are too high (i.e. dragged below the last panel) set to the # of panels to display, or the size of the array list, whichever is smaller
    println(newPanelPlacement);


    if (lastDraggedOverId == -1)
    {

      lastDraggedOverId = newPanelPlacement + currentTopPanel;
      lastDraggedOverColor =   poses.get(newPanelPlacement + currentTopPanel).getLocalColorScheme();
      poses.get(newPanelPlacement + currentTopPanel).setLocalColorScheme(15);

      println("First");
    } else if ((lastDraggedOverId != (newPanelPlacement + currentTopPanel)))
    {

      poses.get(lastDraggedOverId).setLocalColorScheme(lastDraggedOverColor);
      println("change! " +" " + lastDraggedOverColor+ " " + currentTopPanel);

      lastDraggedOverId = newPanelPlacement + currentTopPanel;
      lastDraggedOverColor =   poses.get(newPanelPlacement + currentTopPanel).getLocalColorScheme();
      poses.get(newPanelPlacement + currentTopPanel).setLocalColorScheme(15);
    } else
    {

      //lastDraggedOverId = newPanelPlacement + currentTopPanel;
      //poses.get(newPanelPlacement + currentTopPanel).setLocalColorScheme(0);
    }



    //check is the panel that set the 'dragFlag' has stopped being dragged.
    if (poses.get(dragFlag).isDragging() == false)
    {

      poses.get(lastDraggedOverId).setLocalColorScheme(lastDraggedOverColor);//set color for the displaced panel
      lastDraggedOverId = -1;//reset lastDragged vars for next iteration

      //dragFlag now contains a value corresponding to the the panel that was just being dragged
      //


      int lowestPanel = min(dragPanelNumber, newPanelPlacement); //figure out which panel number is lower



      println("you dragged panel #" + dragPanelNumber+ "to position "+ dragPanelY  +" Which puts it at panel #"+newPanelPlacement);


      //array list management
      tempPanel0 = poses.get(dragPanelNumber);//copy the panel that was being dragged to a temporary object
      poses.remove(dragPanelNumber);//remove the panel from the array list
      poses.add(newPanelPlacement, tempPanel0);//add the panel into the array list at the position of the displaced panel

      int[] tempPoseData0 = poseData.get(dragPanelNumber);//copy the panel that was being dragged to a temporary object
      poseData.remove(dragPanelNumber);
      poseData.add(newPanelPlacement, tempPoseData0);//add the panel into the array list at the position of the displaced panel


      //rebuild all of the list placement based on its correct array placement
      for (int i = lowestPanel; i < poses.size ()-currentTopPanel; i++)
      {
        println("i " + i);
        poses.get(currentTopPanel+i).moveTo(panelsX, panelsYStart + (panelYOffset*i));//move the panel that was being dragged to its new position
        poses.get(currentTopPanel+i).setText(Integer.toString(currentTopPanel+i));//set the text displayed to the same as the new placement
        //whenever the program displaces a panel down, one panel will need to go from being visible to not being visible this will always be the 'numberPanelsDisplay'th panel
        if (i == numberPanelsDisplay)
        {
          poses.get(currentTopPanel+i).setVisible(false);//set the panel that has 'dropped off' the visual plane to not visible
        }
      }




      tempPanel0 = null;
      dragFlag = -1;
      println("reset Flag");
    }//end dragging check.
  }//end dragFlag check






  if (playSequence == true)
  {
    pauseTime = PApplet.parseInt(pauseTextField.getText());

    if (millis() - lastTime > (20 * deltaCurrent + pauseTime))
    {
      //println("50 millis");
      for (int i = 0; i < poses.size (); i++)
      {
        if (i == lastPose)
        {
          poses.get(i).setCollapsed(false);
          poseToWorkspaceInternal(lastPose);
          updateFlag = true;//set update flag to signal sending an update on the next cycle
          updateOffsetCoordinates();//update the coordinates to offset based on the current mode



          println("play"+i);
        } else
        {
          poses.get(i).setCollapsed(true);
        }
      }
      lastPose = lastPose + 1;

      lastTime = millis();

      if (lastPose  >= poses.size())
      {
        //playSequence = false;
        lastPose = 0;
        println("play ending");
      }
    }
  }



}//end draw()


/*****************************************************
 *  stop()
 *
 *  Tasks to perform on end of program
 ******************************************************/
public void stop()
{

  debugOutput.flush(); // Writes the remaining data to the file
  debugOutput.close(); // Finishes the file
}


/******************************************************
 *  prepareExitHandler()
 *
 *  Tasks to perform on end of program
 * https://forum.processing.org/topic/run-code-on-exit
 ******************************************************/
private void prepareExitHandler () {
  Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
    public void run ()
    {
      if (debugFileCreated == true)
      {
        debugOutput.flush(); // Writes the remaining data to the file
        debugOutput.close(); // Finishes the file
      }
    }
  }
  ));
}

public void printlnDebug(String message, int type)
{
  if (debugConsole == true)
  {
    if ((type == 1 & debugGuiEvent == true) || type == 0 || type == 2)
    {
      println(message);
    }
  }

  if (debugFile == true)
  {

    if ((type == 1 & debugGuiEvent == true) || type == 0 || type == 2)
    {

      if (debugFileCreated == false)
      {
        debugOutput = createWriter("debugArmLink.txt");
        debugOutput.println("Started at "+ day() +"-"+ month() +"-"+ year() +" "+ hour() +":"+ minute() +"-"+ second() +"-");
        debugFileCreated = true;
      }


      debugOutput.println(message);
    }
  }
}

//wrapper for printlnDebug(String, int)
//assume normal behavior, message type = 0
public void printlnDebug(String message)
{
  printlnDebug(message, 0);
}

/******************************************************
 *  printlnDebug()
 *
 *  function used to easily enable/disable degbugging
 *  enables/disables debugging to the console
 *  prints normally to the output
 *
 *  Parameters:
 *    String message
 *      string to be sent to the debugging method
 *    int type
 *        Type of event
 *         type 0 = normal program message
 *         type 1 = GUI event
 *         type 2 = serial packet
 *  Globals Used:
 *      boolean debugGuiEvent
 *      boolean debugConsole
 *      boolean debugFile
 *      PrintWriter debugOutput
 *      boolean debugFileCreated
 *  Returns:
 *    void
 ******************************************************/
public void printDebug(String message, int type)
{
  if (debugConsole == true)
  {
    if ((type == 1 & debugGuiEvent == true)  || type == 2)
    {
      print(message);
    }
  }

  if (debugFile == true)
  {

    if ((type == 1 & debugGuiEvent == true) || type == 0 || type == 2)
    {

      if (debugFileCreated == false)
      {
        debugOutput = createWriter("debugArmLink.txt");

        debugOutput.println("Started at "+ day() +"-"+ month() +"-"+ year() +" "+ hour() +":"+ minute() +"-"+ second() );

        debugFileCreated = true;
      }


      debugOutput.print(message);
    }
  }
}

//wrapper for printlnDebug(String, int)
//assume normal behavior, message type = 0
public void printDebug(String message)
{
  printDebug(message, 0);
}

/******************************************************
 * Key combinations
 * holding a number ket and pressing up/down will
 * increment/decrement the corresponding field
 *
 * The program will use keyPressed() to log whenever a
 * number key is being held. If later 'Up' or 'Down'
 * is also logged, they value will be changed
 *  keyReleased() will be used to un-log the number values
 *  once they are released.
 *
 * 1-x/base
 * 2-y/shoulder
 * 3-z/elbow
 * 4-wrist angle
 * 5-wrist rotate
 * 6-gripper
 ******************************************************/
public void keyPressed()
{

  if (currentArm != 0)
  {
    //change 'updageFlag' variable if 'enter' is pressed
    if (key ==ENTER)
    {
      updateFlag = true;
      updateOffsetCoordinates();
    }

    //if any of the numbers 1-6 are currently being pressed, change the state of the variable
    if (key =='1')
    {
      xkey=true;
    }
    if (key =='2')
    {
      ykey=true;
    }
    if (key =='3')
    {
      zkey=true;
    }
    if (key =='5')
    {
      wangkey=true;
    }
    if (key =='6')
    {
      wrotkey=true;
    }
    if (key =='4')
    {
      gkey=true;
    }
    if (key =='7')
    {
      dkey=true;
    }




    if (key == ' ')
    {

      sendCommanderPacket(0, 0, 0, 0, 0, 0, 0, 0, 17);    //send a commander style packet - the first 8 bytes are inconsequntial, only the last byte matters. '17' is the extended byte that will stop the arm
      updateFlag = false;
      autoUpdateCheckbox.setSelected(false);
    }

    if (key == 'p')
    {

      playSequence = !playSequence;
    }





    if (key == ',')
    {

      poseToWorkspaceInternal(currentPose);
    }
    if (key == '.')
    {
      workspaceToPoseInternal();
    }


    //check for up/down keys
    if (key == CODED)
    {

      //if up AND a number 1-6 are being pressed, increment the appropriate field
      if (keyCode == UP)
      {
        if (xkey == true)
        {
          println(xCurrent);
          xCurrent = xCurrent + 1;
        }
        if (ykey == true)
        {
          yCurrent = yCurrent + 1;
        }
        if (zkey == true)
        {
          zCurrent = zCurrent + 1;
        }
        if (wangkey == true)
        {
          wristAngleCurrent = wristAngleCurrent + 1;
        }
        if (wrotkey == true)
        {
          wristRotateCurrent = wristRotateCurrent + 1;
        }
        if (gkey == true)
        {
          gripperCurrent = gripperCurrent + 1;
        }
        if (dkey == true)
        {
          deltaCurrent = deltaCurrent + 1;
        }



        xTextField.setText(Integer.toString(xCurrent));
        yTextField.setText(Integer.toString(yCurrent));
        zTextField.setText(Integer.toString(zCurrent));
        wristAngleTextField.setText(Integer.toString(wristAngleCurrent));
        wristRotateTextField.setText(Integer.toString(wristRotateCurrent));
        gripperTextField.setText(Integer.toString(gripperCurrent));

        if (currentMode == 1)
        {
          xSlider.setValue(xCurrent);
          ySlider.setValue(yCurrent);
          zSlider.setValue(zCurrent);
          wristRotateKnob.setValue(wristRotateCurrent);
          wristAngleKnob.setValue(wristAngleCurrent);
          gripperLeftSlider.setValue(gripperCurrent);
          gripperRightSlider.setValue(gripperCurrent);
          deltaSlider.setValue(deltaCurrent);
        } else if (currentMode == 2 )
        {
          baseKnob.setValue(xCurrent);
          ySlider.setValue(yCurrent);
          zSlider.setValue(zCurrent);
          wristRotateKnob.setValue(wristRotateCurrent);
          wristAngleKnob.setValue(wristAngleCurrent);
          gripperLeftSlider.setValue(gripperCurrent);
          gripperRightSlider.setValue(gripperCurrent);
          deltaSlider.setValue(deltaCurrent);
        } else if (currentMode == 3)
        {
          baseKnob.setValue(xCurrent);
          shoulderKnob.setValue(yCurrent);
          elbowKnob.setValue(zCurrent);
          wristRotateKnob.setValue(wristRotateCurrent);
          wristAngleKnob.setValue(wristAngleCurrent);
          gripperLeftSlider.setValue(gripperCurrent);
          gripperRightSlider.setValue(gripperCurrent);
          deltaSlider.setValue(deltaCurrent);
        }

      }

      //if down AND a number 1-6 are being pressed, increment the appropriate field
      if (keyCode == DOWN)
      {
        if (xkey == true)
        {
          xCurrent = xCurrent - 1;
        }
        if (ykey == true)
        {
          yCurrent = yCurrent - 1;
        }
        if (zkey == true)
        {
          zCurrent = zCurrent - 1;
        }
        if (wangkey == true)
        {
          wristAngleCurrent = wristAngleCurrent - 1;
        }
        if (wrotkey == true)
        {
          wristRotateCurrent = wristRotateCurrent - 1;
        }
        if (gkey == true)
        {
          gripperCurrent = gripperCurrent - 1;
        }
        if (dkey == true)
        {
          deltaCurrent = deltaCurrent - 1;
        }


        xTextField.setText(Integer.toString(xCurrent));
        yTextField.setText(Integer.toString(yCurrent));
        zTextField.setText(Integer.toString(zCurrent));
        wristAngleTextField.setText(Integer.toString(wristAngleCurrent));
        wristRotateTextField.setText(Integer.toString(wristRotateCurrent));
        gripperTextField.setText(Integer.toString(gripperCurrent));

        if (currentMode == 1)
        {
          xSlider.setValue(xCurrent);
          ySlider.setValue(yCurrent);
          zSlider.setValue(zCurrent);
          wristRotateKnob.setValue(wristRotateCurrent);
          wristAngleKnob.setValue(wristAngleCurrent);
          gripperLeftSlider.setValue(gripperCurrent);
          gripperRightSlider.setValue(gripperCurrent);
          deltaSlider.setValue(deltaCurrent);
        } else if (currentMode == 2 )
        {
          baseKnob.setValue(xCurrent);
          ySlider.setValue(yCurrent);
          zSlider.setValue(zCurrent);
          wristRotateKnob.setValue(wristRotateCurrent);
          wristAngleKnob.setValue(wristAngleCurrent);
          gripperLeftSlider.setValue(gripperCurrent);
          gripperRightSlider.setValue(gripperCurrent);
          deltaSlider.setValue(deltaCurrent);
        } else if (currentMode == 3)
        {
          baseKnob.setValue(xCurrent);
          shoulderKnob.setValue(yCurrent);
          elbowKnob.setValue(zCurrent);
          wristRotateKnob.setValue(wristRotateCurrent);
          wristAngleKnob.setValue(wristAngleCurrent);
          gripperLeftSlider.setValue(gripperCurrent);
          gripperRightSlider.setValue(gripperCurrent);
          deltaSlider.setValue(deltaCurrent);
        }

      }


    }
  }
}
public void keyReleased()
{

  //change variable state when number1-6 is released

    if (key =='1')
  {
    xkey=false;
  }
  if (key =='2')
  {
    ykey=false;
  }
  if (key =='3')
  {
    zkey=false;
  }
  if (key =='5')
  {
    wangkey=false;
  }
  if (key =='6')
  {
    wrotkey=false;
  }
  if (key =='4')
  {
    gkey=false;
  }
  if (key =='7')
  {
    dkey = false;
  }
}
/***********************************************************************************
 *  }--\     InterbotiX     /--{
 *      |    Arm Link      |
 *   __/                    \__
 *  |__|                    |__|
 *
 *  arbotix.pde
 *
 *	This file has several functions for interfacing with the ArbotiX robocontroller
 *	using the ArmLink protocol.
 *	See 'ArmLnk.pde' for building this application.
 *
 ***********************************************************************************/


/******************************************************
 *  readFromArm(int, boolean)
 *
 *  reads data back from the ArbotiX/Arm
 *
 *  Normally this is called from readFromArm(int) -
 *  this will block the program and make it wait
 * 'packetRepsonseTimeout' ms. Most of the time the program
 *  will need to wait, as the arm is moving to a position
 *  and will not send a response packet until it has
 *  finished moving to that position.
 *
 *  However this will add a lot of time to the 'autoSearch'
 *  functionality. When the arm starts up it will immediatley send a
 *  ID packet to identify itself so a non-waiting version is
 *  avaialble -  readFromArmFast(int) which is equivalent to
 *  readFromArm(int, false)
 *
 *  Parameters:
 *    int bytesExpected
 *      # of bytes expected in the response packet
 *    boolean wait
 *        Whether or not to wait 'packetRepsonseTimeout' ms for a response
 *         true = wait
 *         false = do not wait
 *  Globals Used:
 *      Serial sPort
 *      long packetRepsonseTimeout
 *
 *  Returns:
 *    byte[]  responseBytes
 *      byte array with response data from ArbotiX/Arm
 ******************************************************/
public byte[] readFromArm(int bytesExpected, boolean wait)
{
  byte[] responseBytes = new byte[bytesExpected];    //byte array to hold response data
  delayMs(100);//wait a minimum 100ms to ensure that the controller has responded - this applies to both wait==true and wait==false conditions

  byte bufferByte = 0;  //current byte that is being read
  long startReadingTime = millis();//time that the program started looking for data

  printDebug("Incoming Raw Packet from readFromArm():",2); //debug

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
      printDebug(hex(bufferByte) + "-",2); //debug
    }
    else
    {
      printDebug("NO BYTE-");//debug
    }
  }//end looking for bytes from packet
  printlnDebug(" ",2); //debug  finish line

  sPorts[armPortIndex].clear();  //clear serial port for the next read

  return(responseBytes);  //return serial data
}


//wrapper for readFromArm(int, boolean)
//assume normal behavior, wait = true
public byte[] readFromArm(int bytesExpected)
{
  return(readFromArm(bytesExpected,true));
}


//wrapper for readFromArm(int, boolean)
//wait = false. Used for autosearch/startup
public byte[] readFromArmFast(int bytesExpected)
{
  return(readFromArm(bytesExpected,false));
}




/******************************************************
 *  verifyPacket(int, boolean)
 *
 *  verifies a packet received from the ArbotiX/Arm
 *
 *  This function will do the following to verify a packet
 *  -calculate a local checksum and compare it to the
 *    transmitted checksum
 *  -check the error byte for any data
 *  -check that the armID is supported by this program
 *
 *  Parameters:
 *    byte[]  returnPacket
 *      byte array with response data from ArbotiX/Arm
 *
 *
 *  Returns:
 *    boolean verifyPacket
 *      true = packet is OK
 *      false = problem with the packet
 *
 *  TODO: -Modify to return specific error messages
 *        -Make the arm ID check modular to facilitate
 *         adding new arms.
 ******************************************************/
public boolean verifyPacket(byte[] returnPacket)
{
  int packetLength = returnPacket.length;  //length of the packet
  int tempChecksum = 0; //int for temporary checksum calculation
  byte localChecksum; //local checksum calculated by processing

  printDebug("Begin Packet Verification of :");
  for(int i = 0; i < packetLength;i++)
  {
    printDebug(returnPacket[i]+":");
  }
  //check header, which should always be 255/0xff
  if(returnPacket[0] == PApplet.parseByte(255))
  {
      //iterate through bytes # 1 through packetLength-1 (do not include header(0) or checksum(packetLength)
      for(int i = 1; i<packetLength-1;i++)
      {
        tempChecksum = PApplet.parseInt(returnPacket[i]) + tempChecksum;//add byte value to checksum
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
            printlnDebug("verifyPacket Success!");
            return(true);
          }
          else {printlnDebug("verifyPacket Error: Invalid Arm Detected! Arm ID:"+returnPacket[1]);}
        }
        else {printlnDebug("verifyPacket Error: Error Packet Reports:"+returnPacket[3]);}
      }
      else {printlnDebug("verifyPacket Error: Checksum does not match: Returned:"+ returnPacket[packetLength-1] +" Calculated:"+localChecksum );}
  }
  else {printlnDebug("verifyPacket Error: No Header!");}

  return(false);

}

/******************************************************
 *  checkArmStartup()
 *
 *  function used to check for the presense of a
 *  ArbotiX/Arm on a serial port.

 *  This function also sets the initial Global 'currentArm'
 *
 *  Parameters:
 *    None
 *
 *  Globals used:
 *    int currentArm
 *
 *  Returns:
 *    boolean
 *      true = arm has been detected on current serial port
 *      false = no arm detected on current serial port
 *
 ******************************************************/
public boolean checkArmStartup()
{
  byte[] returnPacket = new byte[5];  //byte array to hold return packet, which is 5 bytes long
  long startTime = millis();
  long currentTime = startTime;
  printlnDebug("Checking for arm on startup ");
  while(currentTime - startTime < startupWaitTime )
  {
    delayMs(100);  //The ArbotiX has a delay of 50ms between starting the serial continueing the program, include an extra 10ms for other ArbotiX startup tasks
    for(int i = 0; i< sPorts.length;i++)
    {
      if(sPorts[i] != null)
      {
        armPortIndex = i;

        printlnDebug("Checking for arm on startup - index# " + i);
        sendCommanderPacket(0, 200, 200, 0, 512, 256, 128, 0, 112);    //send a commander style packet - the first 8 bytes are inconsequntial, only the last byte matters. '112' is the extended byte that will request an ID packet
        returnPacket = readFromArmFast(5);//read raw data from arm, complete with wait time

        if(verifyPacket(returnPacket) == true)
        {
          currentArm = returnPacket[1]; //set the current arm based on the return packet
          printlnDebug("Startup Arm #" +currentArm+ " Found");
          setPositionParameters();      //set the GUI default/min/maxes and field lables

          return(true) ;                //Return a true signal to signal that an arm has been found
        }
      }
    }

    currentTime = millis();
  }
  armPortIndex = -1;
  return(false);


}


/******************************************************
 *  isArmConnected()
 *
 *  generic function to check for the presence of an arm
 *  during normal operation.
 *
 *  Parameters:
 *    None
 *
 *  Globals used:
 *    int currentArm
 *
 *  Returns:
 *    boolean
 *      true = arm has been detected on current serial port
 *      false = no arm detected on current serial port
 *
 ******************************************************/
public boolean isArmConnected()
{
  byte[] returnPacket = new byte[5];//return id packet is 5 bytes long

  printlnDebug("Checking for arm -  sending packet");
  sendCommanderPacket(0, 200, 200, 0, 512, 256, 128, 0, 112);    //send a commander style packet - the first 8 bytes are inconsequntial, only the last byte matters. '112' is the extended byte that will request an ID packet

  returnPacket = readFromArm(5);//read raw data from arm, complete with wait time

  if(verifyPacket(returnPacket) == true)
  {
    printlnDebug("Arm Found");
    return(true) ;
  }
  else
  {
    printlnDebug("No Arm Found");
    return(false);
  }
}

/******************************************************
 *  putArmToSleep()
 *
 *  function to put the arm to sleep. This will move
 *  the arm to a 'rest' position and then turn the
 * torque off for the servos
 *
 *  Parameters:
 *    None
 *
 *
 *  Returns:
 *    boolean
 *      true = arm has been put to sleep
 *      false = no return packet was detected from the arm.
 *
 ******************************************************/
public boolean putArmToSleep()
{
  printDebug("Attempting to put arm in sleep mode - ");
  sendCommanderPacket(0,0,0,0,0,0,0,0,96);//only the last/extended byte matters - 96 signals the arm to go to sleep

  byte[] returnPacket = new byte[5];//return id packet is 5 bytes long
  returnPacket = readFromArm(5);//read raw data from arm
  if(verifyPacket(returnPacket) == true)
  {
    printlnDebug("Sleep mode success!");
    return(true) ;
  }
  else
  {
    printlnDebug("Sleep mode-No return packet detected");
    displayError("There was a problem putting the arm in sleep mode","");
    return(false);
  }
}


/******************************************************
 *  changeArmMode()
 *
 *  sends a packet to set the arms mode and orientation
 *  based on the global mode and orientation values
 *  This function will send a packet with the extended
 *  byte coresponding to the correct IK mode and wrist
 *  orientation. The arm will move from its current
 *  position to the 'home' position for the current
 *  mode.
 *  Backhoe mode does not have different straight/
 *  90 degree modes.
 *
 *  Extended byte - Mode
 *  32 - cartesian, straight mode
 *  40 - cartesian, 90 degree mode
 *  48 - cylindrical, straight mode
 *  56 - cylindrical, 90 degree mode
 *  64 - backhoe
 *
 *  Parameters:
 *    None
 *
 *  Globals used:
 *    currentMode
 *    currentOrientation
 *
 *  Returns:
 *    boolean
 *      true = arm has been put in the mode correctly
 *      false = no return packet was detected from the arm.
 *
 ******************************************************/
public boolean changeArmMode()
{

  byte[] returnPacket = new byte[5];//return id packet is 5 bytes long

 //switch based on the current mode
 switch(currentMode)
  {
    //cartesian mode case
    case 1:
      //switch based on the current orientation
      switch(currentOrientation)
      {
        case 1:
          sendCommanderPacket(0,0,0,0,0,0,0,0,32);//only the last/extended byte matters, 32 = cartesian, straight mode
          printDebug("Setting Arm to Cartesian IK mode, Gripper Angle Straight - ");
          break;
        case 2:
          sendCommanderPacket(0,0,0,0,0,0,0,0,40);//only the last/extended byte matters, 40 = cartesian, 90 degree mode
          printDebug("Setting Arm to Cartesian IK mode, Gripper Angle 90 degree - ");
          break;
      }//end orientation switch
      break;//end cartesian mode case

    //cylindrical mode case
    case 2:
      //switch based on the current orientation
      switch(currentOrientation)
      {
        case 1:
          sendCommanderPacket(0,0,0,0,0,0,0,0,48);//only the last/extended byte matters, 48 = cylindrical, straight mode
          printDebug("Setting Arm to Cylindrical IK mode, Gripper Angle Straight - ");
          break;
        case 2:
          sendCommanderPacket(0,0,0,0,0,0,0,0,56);//only the last/extended byte matters, 56 = cylindrical, 90 degree mode
          printDebug("Setting Arm to Cylindrical IK mode, Gripper Angle 90 degree - ");
          break;
      }//end orientation switch
      break;//end cylindrical mode case

    //backhoe mode case
    case 3:
      sendCommanderPacket(0,0,0,0,0,0,0,0,64);//only the last/extended byte matters, 64 = backhoe
          printDebug("Setting Arm to Backhoe IK mode - ");
      break;//end backhoe mode case
  }

  returnPacket = readFromArm(5);//read raw data from arm
  if(verifyPacket(returnPacket) == true)
  {
    printlnDebug("Response succesful! Arm mode changed");
    return(true) ;
  }
  else
  {
    printlnDebug("No Response - Failure?");

    displayError("There was a problem setting the arm mode","");

    return(false);
  }

}

/******************************************************
 *  delayMs(int)
 *
 *  function waits/blocks the program for 'ms' milliseconds
 *  Used for very short delays where the program only needs
 *  to wait and does not need to execute code
 *
 *  Parameters:
 *    int ms
 *      time, in milliseconds to wait
 *  Returns:
 *    void
 ******************************************************/
public void delayMs(int ms)
{

  int time = millis();  //time that the program starts the loop
  while(millis()-time < ms)
  {
     //loop/do nothing until the different between the current time and 'time'
  }
}


/******************************************************
 *  sendCommanderPacket(int, int, int, int, int, int, int, int, int)
 *
 *  This function will send a commander style packet
 *  the ArbotiX/Arm. This packet has 9 bytes and includes
 *  positional data, button data, and extended instructions.
 *  This function is often used with the function
 *  readFromArm()
 *  to verify the packet was received correctly
 *
 *  Parameters:
 *    int x
 *      offset X value (cartesian mode), or base value(Cylindrical and backhoe mode) - will be converted into 2 bytes
 *    int y
 *        Y Value (cartesian and cylindrical mode) or shoulder value(backhoe mode) - will be converted into 2 bytes
 *    int z
 *        Z Value (cartesian and cylindrical mode) or elbow value(backhoe mode) - will be converted into 2 bytes
 *    int wristAngle
 *      offset wristAngle value(cartesian and cylindrical mode) or wristAngle value (backhoe mode) - will be converted into 2 bytes
 *    int wristRotate
 *      offset wristRotate value(cartesian and cylindrical mode) or wristRotate value (backhoe mode) - will be converted into 2 bytes
 *    int gripper
 *      Gripper Value(All modes) - will be converted into 2 bytes
 *    int delta
 *      delta(speed) value (All modes) - will be converted into 1 byte
 *    int button
 *      digital button values (All modes) - will be converted into 1 byte
 *    int extended
 *       value for extended instruction / special instruction - will be converted into 1 byte
 *
 *  Global used: sPort
 *
 *  Return:
 *    Void
 *
 ******************************************************/
public void sendCommanderPacket(int x, int y, int z, int wristAngle, int wristRotate, int gripper, int delta, int button, int extended)
{
   sPorts[armPortIndex].clear();//clear the serial port for the next round of communications

  //convert each positional integer into 2 bytes using intToBytes()
  byte[] xValBytes = intToBytes(x);
  byte[] yValBytes = intToBytes(y);
  byte[] zValBytes =  intToBytes(z);
  byte[] wristRotValBytes = intToBytes(wristRotate);
  byte[] wristAngleValBytes = intToBytes(wristAngle);
  byte[] gripperValBytes = intToBytes(gripper);
  //cast int to bytes
  byte buttonByte = PApplet.parseByte(button);
  byte extValByte = PApplet.parseByte(extended);
  byte deltaValByte = PApplet.parseByte(delta);
  boolean flag = true;
  //calculate checksum - add all values, take lower byte (%256) and invert result (~). you can also invert results by (255-sum)
  byte checksum = (byte)(~(xValBytes[1]+xValBytes[0]+yValBytes[1]+yValBytes[0]+zValBytes[1]+zValBytes[0]+wristAngleValBytes[1]+wristAngleValBytes[0]+wristRotValBytes[1]+wristRotValBytes[0]+gripperValBytes[1]+gripperValBytes[0]+deltaValByte + buttonByte+extValByte)%256);

  //send commander style packet. Following labels are for cartesian mode, see function comments for clyindrical/backhoe mode
    //try to write the first header byte
    try
    {
      sPorts[armPortIndex].write(0xff);//header
    }
    //catch an exception in case of serial port problems
    catch(Exception e)
    {
       printlnDebug("Error: packet not sent: " + e + ": 0xFF 0x" +hex(xValBytes[1]) +" 0x" +hex(xValBytes[0]) +" 0x" +hex(yValBytes[1]) +" 0x" +hex(yValBytes[0])+" 0x" +hex(zValBytes[1])+" 0x" +hex(zValBytes[0]) +" 0x" +hex(wristAngleValBytes[1]) +" 0x" +hex(wristAngleValBytes[0]) +" 0x" + hex(wristRotValBytes[1])+" 0x" +hex(wristRotValBytes[0]) +" 0x" + hex(gripperValBytes[1])+" 0x" + hex(gripperValBytes[0])+" 0x" + hex(deltaValByte)+" 0x" +hex(buttonByte) +" 0x" +hex(extValByte) +" 0x"+hex(checksum) +"",2);
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
      printlnDebug("Packet Sent: 0xFF 0x" +hex(xValBytes[1]) +" 0x" +hex(xValBytes[0]) +" 0x" +hex(yValBytes[1]) +" 0x" +hex(yValBytes[0])+" 0x" +hex(zValBytes[1])+" 0x" +hex(zValBytes[0]) +" 0x" +hex(wristAngleValBytes[1]) +" 0x" +hex(wristAngleValBytes[0]) +" 0x" + hex(wristRotValBytes[1])+" 0x" +hex(wristRotValBytes[0]) +" 0x" + hex(gripperValBytes[1])+" 0x" + hex(gripperValBytes[0])+" 0x" + hex(deltaValByte)+" 0x" +hex(buttonByte) +" 0x" +hex(extValByte) +" 0x"+hex(checksum) +"",2);
    }


}

/******************************************************
 *  intToBytes(int)
 *
 *  This function will take an interger and convert it
 *  into two bytes. These bytes can then be easily
 *  transmitted to the ArbotiX/Arm. Byte[0] is the low byte
 *  and Byte[1] is the high byte
 *
 *  Parameters:
 *    int convertInt
 *      integer to be converted to bytes
 *  Return:
 *    byte[]
 *      byte array with two bytes Byte[0] is the low byte and Byte[1]
 *      is the high byte
 ******************************************************/
public byte[] intToBytes(int convertInt)
{
  byte[] returnBytes = new byte[2]; // array that holds the two bytes to return
  byte mask = PApplet.parseByte(255);          //mask for the low byte (255/0xff)
  returnBytes[0] =PApplet.parseByte(convertInt & mask);//low byte - perform an '&' operation with the byte mask to remove the high byte
  returnBytes[1] =PApplet.parseByte((convertInt>>8) & mask);//high byte - shift the byte to the right 8 bits. perform an '&' operation with the byte mask to remove any additional data
  return(returnBytes);  //return byte array

}

/******************************************************
 *  bytesToInt(byte[])
 *
 *  Take two bytes and convert them into an integer
 *
 *  Parameters:
 *    byte[] convertBytes
 *      bytes to be converted to integer
 *  Return:
 *    int
 *      integer value from 2 butes
 ******************************************************/
public int bytesToInt(byte[] convertBytes)
{
  return((PApplet.parseInt(convertBytes[1]<<8))+PApplet.parseInt(convertBytes[0]));//shift high byte up 8 bytes, and add it to the low byte. cast to int to ensure proper signed/unsigned behavior
}

/****************
 *  updateOffsetCoordinates()
 *
 *  modifies the current global coordinate
 *  with an appropriate offset
 *
 *  As the Arm Link software communicates in
 *  unsigned bytes, any value that has negative
 *  values in the GUI must be offset. This function
 *  will add the approprate offsets based on the
 *  current mode of operation( global variable 'currentMode')
 *
 *  Parameters:
 *    None:
 *  Globals used:
 *    'Current' position vars
 *    'CurrentOffset' position vars
 *  Return:
 *    void
 ***************/

public void  updateOffsetCoordinates()
{
  //offsets are applied based on current mode
  switch(currentMode)
    {
       case 1:
         //x, wrist angle, and wrist rotate must be offset, all others are normal
         xCurrentOffset = xCurrent + 512;
         yCurrentOffset = yCurrent;
         zCurrentOffset = zCurrent;
         wristAngleCurrentOffset =  wristAngleCurrent + 90;
         //wristRotateCurrentOffset = wristRotateCurrent + 512;
         wristRotateCurrentOffset = wristRotateCurrent;
         gripperCurrentOffset = gripperCurrent;
         deltaCurrentOffset = deltaCurrent;
         break;

       case 2:

         //wrist angle, and wrist rotate must be offset, all others are normal
         xCurrentOffset = xCurrent;
         yCurrentOffset = yCurrent;
         zCurrentOffset = zCurrent;
         wristAngleCurrentOffset =  wristAngleCurrent + 90;
         //wristRotateCurrentOffset = wristRotateCurrent + 512;
         wristRotateCurrentOffset = wristRotateCurrent;
         gripperCurrentOffset = gripperCurrent;
         deltaCurrentOffset = deltaCurrent;
         break;

       case 3:

         //no offsets needed
         xCurrentOffset = xCurrent;
         yCurrentOffset = yCurrent;
         zCurrentOffset = zCurrent;
         wristAngleCurrentOffset =  wristAngleCurrent;
         wristRotateCurrentOffset = wristRotateCurrent;
         gripperCurrentOffset = gripperCurrent;
         deltaCurrentOffset = deltaCurrent;
        break;
    }
}

/****************
 *  updateButtonByte()
 *
 *
 *
 *  Parameters:
 *    None:
 *  Globals used:
 *    int[] digitalButtons
 *    int digitalButtonByte
 *  Return:
 *    void
 ***************/

public void updateButtonByte()
{
  digitalButtonByte = 0;
   for(int i=0;i<8;i++)
  {
    if(digitalButtons[i] == true)
    {
      digitalButtonByte += pow(2,i);
    }
  }
}


//TODO//
public boolean getArmInfo()
{
  return(true);

}




public int analogRead(int analogPort)
{
  byte[] returnPacket = new byte[5];  //byte array to hold return packet, which is 5 bytes long
  int analog = 0;
  printlnDebug("sending request for anlaog 1");
  int analogExtentded = 200 + analogPort;
  sendCommanderPacket(xCurrentOffset, yCurrentOffset, zCurrentOffset, wristAngleCurrentOffset, wristRotateCurrentOffset, gripperCurrentOffset, deltaCurrentOffset, digitalButtonByte, analogExtentded);    //send a commander style packet - the first 8 bytes are inconsequntial, only the last byte matters. '112' is the extended byte that will request an ID packet
  returnPacket = readFromArmFast(5);//read raw data from arm, complete with wait time
  byte[] analogBytes = {returnPacket[3],returnPacket[2]};
  analog = bytesToInt(analogBytes);

  printlnDebug("Return Packet" + PApplet.parseInt(returnPacket[0]) + "-" +  PApplet.parseInt(returnPacket[1]) + "-"  + PApplet.parseInt(returnPacket[2]) + "-"  + PApplet.parseInt(returnPacket[3]) + "-"  + PApplet.parseInt(returnPacket[4]));
  printlnDebug("analog value: " + analog);

  return(analog);

}


/***********************************************************************************
 *  }--\     InterbotiX     /--{
 *      |    Arm Link      |
 *   __/                    \__
 *  |__|                    |__|
 *
 *  global.pde
 *
 *  This file has several global variables relating to the positional data for the arms.
 *  See 'ArmControl.pde' for building this application.
 *
 *
 * The following variables are named for Cartesian mode -
 * however the data that will be held/sent will vary based on the current IK mode
 ****************************************************************************
 * Variable name | Cartesian Mode | Cylindrcal Mode | Backhoe Mode          |
 *_______________|________________|_________________|_______________________|
 *   x           |   x            |   base          |   base joint          |
 *   y           |   y            |   y             |   shoulder joint      |
 *   z           |   z            |   z             |   elbow joint         |
 *   wristAngle  |  wristAngle    |  wristAngle     |   wrist angle joint   |
 *   wristRotate |  wristeRotate  |  wristeRotate   |   wrist rotate jount  |
 *   gripper     |  gripper       |  gripper        |   gripper joint       |
 *   delta       |  delta         |  delta          |   n/a                 |
********************************************************************************/


//WORKING POSITION VARIABLES

//default values and min/max , {default, min, max}
//initially set to values for pincher in normal mode which should be safe for most arms (this shouldn't matter, as these values will get changed when an arm is connected)
//these parameters will be loaded based on the 1)Arm type 2)IK mode 3)Wrist Angle Orientation
int[] xParameters = {0,-200,200};//
int[] yParameters = {200,50,240};
int[] zParameters = {200,20,250};
int[] wristAngleParameters = {0,-90,90};
int[] wristRotateParameters = {0,-512,511};
int[] gripperParameters = {256,0,512};
int[] deltaParameters = {125,0,256};
int[] pauseParameters = {1000,0,10000};

//values for the current value directly from the GUI element. These are updated by the slider/text boxes
int xCurrent = xParameters[0]; //current x value in text field/slider
int yCurrent = yParameters[0]; //current y value in text field/slider
int zCurrent = zParameters[0]; //current z value in text field/slider
int wristAngleCurrent = wristAngleParameters[0]; //current Wrist Angle value in text field/slider
int wristRotateCurrent = wristRotateParameters[0]; //current  Wrist Rotate value in text field/slider
int gripperCurrent = gripperParameters[0]; //current Gripper value in text field/slider
int deltaCurrent = deltaParameters[0]; //current delta value in text field/slider};
int pauseCurrent = 1000;

//offset values to be send to the ArbotiX/Arm. whether or not these values get offsets depends on the current mode
//it will be possible for the 'Current' value to be the same as the 'currentOffset' value.
// see updateOffsetCoordinates()
int xCurrentOffset = xParameters[0]; //current x value to be send to ArbotiX/Arm
int yCurrentOffset = yParameters[0]; //current y value to be send to ArbotiX/Arm
int zCurrentOffset = zParameters[0]; //current z value to be send to ArbotiX/Arm
int wristAngleCurrentOffset = wristAngleParameters[0]; //current Wrist Angle value to be send to ArbotiX/Arm
int wristRotateCurrentOffset = wristRotateParameters[0]; //current  Wrist Rotate value to be send to ArbotiX/Arm
int gripperCurrentOffset = gripperParameters[0]; //current Gripper value to be send to ArbotiX/Arm
int deltaCurrentOffset = deltaParameters[0]; //current delta value to be send to ArbotiX/Arm

boolean[] digitalButtons = {false,false,false,false,false,false,false,false};  //array of 8 boolean to hold the current states of the checkboxes that correspond to the digital i/o
int digitalButtonByte;//int will hold the button byte (will be cast to byte later)

int extendedByte = 0;  //extended byte for special instructions


//END WORKING POSITION VARIABLES

//DEFAULT ARM PARAMETERS

int numberOfArms = 5;

 //XYZ
int[][] armParam0X = new int[numberOfArms][3];
int[][] armParam0Y = new int[numberOfArms][3];
int[][] armParam0Z = new int[numberOfArms][3];
int[][] armParam0WristAngle = new int[numberOfArms][3];
int[][] armParam0WristRotate = new int[numberOfArms][3];

int[][] armParam90X = new int[numberOfArms][3];
int[][] armParam90Y = new int[numberOfArms][3];
int[][] armParam90Z = new int[numberOfArms][3];
int[][] armParam90WristAngle = new int[numberOfArms][3];
int[][] armParam90WristRotate = new int[numberOfArms][3];



int[][] armParamBase = new int[numberOfArms][3];
int[][] armParamBHShoulder = new int[numberOfArms][3];
int[][] armParamBHElbow = new int[numberOfArms][3];
int[][] armParamBHWristAngle = new int[numberOfArms][3];
int[][] armParamBHWristRot = new int[numberOfArms][3];


int[][] armParamGripper = new int[numberOfArms][3];


int[][] armParamWristAngle0Knob = new int[numberOfArms][2];
int[][] armParamWristAngle90Knob = new int[numberOfArms][2];
int[][] armParamWristAngleBHKnob = new int[numberOfArms][2];
int[][] armParamWristRotKnob= new int[numberOfArms][2];

int[][] armParamBaseKnob = new int[numberOfArms][2];
int[][] armParamElbowKnob = new int[numberOfArms][2];
int[][] armParamShoulderKnob = new int[numberOfArms][2];
float[] armParamElbowKnobRotation = new float[numberOfArms];


//default values for the phantomX pincher. These will be loaded into the working position variables
//when the pincher is connected, and when modes are changed.
int[] pincherNormalX = {0,-200,200};
int[] pincherNormalY = {170,50,240};
int[] pincherNormalZ = {210,20,250};
int[] pincherNormalWristAngle = {0,-30,30};
int[] pincherWristRotate = {0,0,0};//not implemented in hardware
int[] pincherGripper = {256,0,512};
int[] pincher90X = {0,-200,200};
int[] pincher90Y = {140,20,150};
int[] pincher90Z = {30,10,150};
int[] pincher90WristAngle = {-90,-90,-45};
int[] pincherBase = {512,1023,0};
int[] pincherBHShoulder = {512,815,205};
int[] pincherBHElbow = {512,1023,205};
int[] pincherBHWristAngle = {512,815,205};
int[] pincherBHWristRot = {512,0,1023};

int[] pincherBHWristAngleNormalKnob = {150,210};//angle data for knob limits
int[] pincherBHWristAngle90Knob = {90,45};//angle data for knob limits

int[] pincherWristAngleBHKnob = {90,270};//angle data for knob limits
int[] pincherWristRotKnob = {120,60};

int[] pincherBaseKnob = {120,60};
int[] pincherShoulderKnob = {180,0};
int[] pincherElbowKnob = {180,60};
float pincherElbowKnobRotation = -PI*1/3;



//default values for the phantomX reactor. These will be loaded into the working position variables
//when the reactor is connected, and when modes are changed.
int[] reactorNormalX = {0,-300,300};
int[] reactorNormalY = {235,50,350};
int[] reactorNormalZ = {210,20,250};
int[] reactorNormalWristAngle = {0,-30,30};
//int[] reactorWristRotate = {0,511,-512};
int[] reactorWristRotate = {512,0,1023};
int[] reactorGripper = {256,0,512};
int[] reactor90X = {0,-300,300};
int[] reactor90Y = {140,20,150};
int[] reactor90Z = {30,10,150};
int[] reactor90WristAngle = {-90,-90,-45};
int[] reactorBase = {512,1023,0};
int[] reactorBHShoulder = {512,810,205};
int[] reactorBHElbow = {512,210,900};
int[] reactorBHWristAngle = {512,200,830};
int[] reactorBHWristRot = {512,1023,0};

int[] reactorWristAngleNormalKnob = {150,210};//angle data for knob limits
int[] reactorWristAngle90Knob = {90,135};//angle data for knob limits
int[] reactorWristAngleBHKnob = {90,270};//angle data for knob limits
int[] reactorWristRotKnob = {120,60};
int[] reactorBaseKnob = {120,60};
int[] reactorShoulderKnob = {180,0};
int[] reactorElbowKnob = {180,30};
float reactorElbowKnobRotation = 0;



//default values for the widowx. These will be loaded into the working position variables
//when the widowx is connected, and when modes are changed.
int[] widowNormalX = {0,-300,300};
int[] widowNormalY = {250,50,400};
int[] widowNormalZ = {225,20,350};
int[] widowNormalWristAngle = {0,-30,30};
int[] widowWristRotate = {512,0,1023};
int[] widowGripper = {256,0,512};
int[] widow90X = {0,-300,300};
int[] widow90Y = {150,20,250};
int[] widow90Z = {30,10,200};
int[] widow90WristAngle = {-90,-90,-45};
int[] widowBase = {2048,4095,0};
int[] widowBHShoulder = {2048,3072,1024};
int[] widowBHElbow = {2048,1024,3072};
int[] widowBHWristAngle = {2048,1024,3072};
int[] widowBHWristRot = {512,1023,0};

int[] widowBHWristAngleNormalKnob = {150,210};//angle data for knob limits
int[] widowBHWristAngle90Knob = {90,135};//angle data for knob limits

int[] widowWristAngleBHKnob = {90,270};//angle data for knob limits
int[] widowWristRotKnob = {120,60};

int[] widowBaseKnob = {90,90};
int[] widowShoulderKnob = {180,0};
int[] widowElbowKnob =  {180,0};
float widowElbowKnobRotation = 0;//-PI*1/3;



//default values for the RobotGeek Snapper. These will be loaded into the working position variables
//when the snapper is connected, and when modes are changed.
int[] snapperNormalX = {0,-150,150};
int[] snapperNormalY = {150,55,200}; //the limits are really 50-200, but 50-54 cause a problem when height Z is maximum
int[] snapperNormalZ = {150,20,225};
int[] snapperNormalWristAngle = {0,-30,30};


int[] snapperWristRotate = {0,0,0};  //Not Implemented in Snapper hardware
int[] snapperGripper = {256,0,512};
int[] snapper90X = {0,-200,200};          //Not Implemented in Snapper firmware
int[] snapper90Y = {140,20,150};          //Not Implemented in Snapper firmware
int[] snapper90Z = {30,10,150};          //Not Implemented in Snapper firmware
int[] snapper90WristAngle = {-90,-90,-45};          //Not Implemented in Snapper firmware
int[] snapperBase = {512,0,1023};          //Not Implemented in Snapper firmware
int[] snapperBHShoulder = {512,205,815};          //Not Implemented in Snapper firmware
int[] snapperBHElbow = {512,205,1023};          //Not Implemented in Snapper firmware
int[] snapperBHWristAngle = {512,205,815};          //Not Implemented in Snapper firmware
int[] snapperBHWristRot = {512,0,1023};          //Not Implemented in Snapper firmware

int[] snapperBHWristAngleNormalKnob = {150,210};//angle data for knob limits
int[] snapperBHWristAngle90Knob = {90,45};//angle data for knob limits

int[] snapperWristAngleBHKnob = {270,90};//angle data for knob limits
int[] snapperWristRotKnob = {120,60};

int[] snapperBaseKnob = {120,60};
int[] snapperShoulderKnob = {120,60};
int[] snapperElbowKnob = {120,60};
float snapperElbowKnobRotation = -PI*1/3;

public void poseToWorkspaceInternal(int pose)
{

int mask = 0;

xCurrent = poseData.get(pose)[0];//set the value that will be sent
xTextField.setText(Integer.toString(xCurrent));//set the text field
xSlider.setValue(xCurrent);//set gui elemeent to same value
baseKnob.setValue(xCurrent);


yCurrent = poseData.get(pose)[1];//set the value that will be sent
yTextField.setText(Integer.toString(yCurrent));//set the text field
ySlider.setValue(yCurrent);//set gui elemeent to same value
shoulderKnob.setValue(yCurrent);

zCurrent = poseData.get(pose)[2];//set the value that will be sent
zTextField.setText(Integer.toString(zCurrent));//set the text field
zSlider.setValue(zCurrent);//set gui elemeent to same value
elbowKnob.setValue(zCurrent);

wristAngleCurrent = poseData.get(pose)[3];//set the value that will be sent
wristAngleTextField.setText(Integer.toString(wristAngleCurrent));//set the text field
wristAngleKnob.setValue(wristAngleCurrent);//set gui elemeent to same value

wristRotateCurrent = poseData.get(pose)[4];//set the value that will be sent
wristRotateTextField.setText(Integer.toString(wristRotateCurrent));//set the text field
wristRotateKnob.setValue(wristRotateCurrent);//set gui elemeent to same value

gripperCurrent = poseData.get(pose)[5];//set the value that will be sent
gripperTextField.setText(Integer.toString(gripperCurrent));//set the text field
gripperSlider.setValue(gripperCurrent);//set gui elemeent to same value
gripperLeftSlider.setValue(gripperCurrent);//set gui elemeent to same value
gripperRightSlider.setValue(gripperCurrent);//set gui elemeent to same value

deltaCurrent = poseData.get(pose)[6];//set the value that will be sent
deltaTextField.setText(Integer.toString(deltaCurrent));//set the text field
deltaSlider.setValue(deltaCurrent);//set gui elemeent to same value

pauseCurrent = poseData.get(pose)[8];
pauseTextField.setText(Integer.toString(pauseCurrent));//set the text field

//extendedByte

int buttonByteFromPose = poseData.get(pose)[7];

 //I'm sure there's a better way to do this
  for (int i = 7; i>=0;i--)
  {
    //subtract 2^i from the button byte, if the value is non-negative, then that byte was active
    if(buttonByteFromPose - pow(2,i) >= 0 )
    {
      buttonByteFromPose = buttonByteFromPose - PApplet.parseInt(pow(2,i));
      switch(i)
      {
        case 0:
        digitalCheckbox1.setSelected(true);
        digitalButtons[1] = true;
        break;

        case 1:
        digitalCheckbox2.setSelected(true);
        digitalButtons[2] = true;
        break;

        case 2:
        digitalCheckbox3.setSelected(true);
        digitalButtons[3] = true;
        break;

        case 3:
        digitalCheckbox4.setSelected(true);
        digitalButtons[4] = true;
        break;

        case 4:
        digitalCheckbox5.setSelected(true);
        digitalButtons[5] = true;
        break;

        case 5:
        digitalCheckbox6.setSelected(true);
        digitalButtons[6] = true;
        break;

        case 6:
        digitalCheckbox7.setSelected(true);
        digitalButtons[7] = true;
        break;
        /*
        case 7:
        digitalCheckbox7.setSelected(true);
        digitalButtons[7] = true;
        break;
        */
      }

   }
   else
   {
     switch(i)
      {
        case 0:
        digitalCheckbox1.setSelected(false);
        digitalButtons[1] = false;
        break;

        case 1:
        digitalCheckbox2.setSelected(false);
        digitalButtons[2] = false;
        break;

        case 2:
        digitalCheckbox3.setSelected(false);
        digitalButtons[3] = false;
        break;

        case 3:
        digitalCheckbox4.setSelected(false);
        digitalButtons[4] = false;
        break;

        case 4:
        digitalCheckbox5.setSelected(false);
        digitalButtons[5] = false;
        break;

        case 5:
        digitalCheckbox6.setSelected(false);
        digitalButtons[6] = false;
        break;

        case 6:
        digitalCheckbox7.setSelected(false);
        digitalButtons[7] = false;
        break;
        /*
        case 7:
        digitalCheckbox7.setSelected(false);
        digitalButtons[7] = false;
        break;
        */
      }

   }


 }


}
//#######################################################################//

  sequencePanel.addControl(movePosesDown);
  sequencePanel.addControl(newPose);
  sequencePanel.addControl(movePosesUp);
  sequencePanel.addControl(poseToWorkspace);
  sequencePanel.addControl(workspaceToPose);
  //sequencePanel.addControl(analog1);
  sequencePanel.addControl(playButton);
  sequencePanel.addControl(stopButton);
  //sequencePanel.addControl(poses.get(0));

  sequencePanel.addControl(savePosesButton);
  sequencePanel.addControl(loadPosesButton);
  sequencePanel.addControl(emergencyStopButton);

  // sequencePanel.addControl(pauseTextField);
  // sequencePanel.addControl(pauseLabel);

  controlPanel.addControl(pauseTextField);
  controlPanel.addControl(pauseLabel);





  controlPanel.addControl(xTextField);
  controlPanel.addControl(xSlider);
  controlPanel.addControl(yTextField);
  controlPanel.addControl(ySlider);
  controlPanel.addControl(yLabel);
  controlPanel.addControl(xLabel);
  controlPanel.addControl(zTextField);
  controlPanel.addControl(zSlider);
  controlPanel.addControl(zLabel);
  controlPanel.addControl(wristAngleTextField);
  controlPanel.addControl(wristAngleSlider);
  controlPanel.addControl(wristAngleLabel);
  controlPanel.addControl(wristRotateTextField);
  controlPanel.addControl(wristRotateSlider);
  controlPanel.addControl(wristRotateLabel);
  controlPanel.addControl(gripperTextField);
  controlPanel.addControl(gripperSlider);
  controlPanel.addControl(gripperLabel);
  controlPanel.addControl(deltaTextField);
  controlPanel.addControl(deltaSlider);
  controlPanel.addControl(deltaLabel);
  controlPanel.addControl(extendedTextField);
  controlPanel.addControl(extendedLabel);
  controlPanel.addControl(autoUpdateCheckbox);
  controlPanel.addControl(updateButton);
  controlPanel.addControl(baseKnob);
  controlPanel.addControl(shoulderKnob);
  controlPanel.addControl(elbowKnob);
  controlPanel.addControl(wristAngleKnob);
  controlPanel.addControl(wristRotateKnob);
  controlPanel.addControl(gripperLeftSlider);
  controlPanel.addControl(gripperRightSlider);
  //controlPanel.addControl(waitingButton);


  ioPanel.addControl(digitalsLabel);
  ioPanel.addControl(digitalCheckbox0);
  ioPanel.addControl(digitalCheckbox1);
  ioPanel.addControl(digitalCheckbox2);
  ioPanel.addControl(digitalCheckbox3);
  ioPanel.addControl(digitalCheckbox4);
  ioPanel.addControl(digitalCheckbox5);
  ioPanel.addControl(digitalCheckbox6);
  ioPanel.addControl(digitalCheckbox7);

  ioPanel.addControl(cameraCheckbox);
  ioPanel.addControl(cameraLabel);


  ioPanel.addControl(analogTextLabel);
  ioPanel.addControl(analogCheckbox);
  ioPanel.addControl(analogLabel[0]);
  ioPanel.addControl(analogLabel[1]);
  ioPanel.addControl(analogLabel[2]);
  ioPanel.addControl(analogLabel[3]);
  ioPanel.addControl(analogLabel[4]);
  ioPanel.addControl(analogLabel[5]);
  ioPanel.addControl(analogLabel[6]);
  ioPanel.addControl(analogLabel[7]);






  waitingButton = new GImageButton(this, 115, 408, 100, 30, new String[] {
    "moving.jpg", "moving.jpg", "moving.jpg"
  }
  );
  waitingButton.setAlpha(0);


//settings
  settingsPanel = new GPanel(this, 10, 280, 230, 230, "Settings Panel");
  settingsPanel.setText("Error Panel");
  settingsPanel.setLocalColorScheme(GCScheme.CYAN_SCHEME);
  settingsPanel.setOpaque(true);
  settingsPanel.setVisible(false);
  settingsPanel.addEventHandler(this, "settingsPanel_Click");
  //settingsPanel.setDraggable(false);
  settingsPanel.setCollapsible(false);


  fileDebugCheckbox = new GCheckbox(this, 5, 20, 200, 20);
  fileDebugCheckbox.setOpaque(false);
  fileDebugCheckbox.addEventHandler(this, "fileDebugCheckbox_change");
  fileDebugCheckbox.setText("Debug to File");

  settingsDismissButton = new GButton(this, 15, 120, 50, 20);
  settingsDismissButton.setText("Done");
  settingsDismissButton.addEventHandler(this, "settingsDismissButton_click");
  settingsDismissButton.setLocalColorScheme(GCScheme.RED_SCHEME);

  settingsPanel.addControl(settingsDismissButton);
  settingsPanel.addControl(fileDebugCheckbox);







  logoImg = loadImage("armLinkLogo.png");  // Load the image into the program
  footerImg = loadImage("footer.png");  // Load the image into the program




  //MOVE TO GUI


}



/******************************************************
 *  wristRotateTextFieldameters()
 *  This function will load the approriate position
 *  defaults, minimums, and maximums into the GUI
 *  text fields and sliders, as well as the limit check
 *  logic. In addition, label elements will be changed
 *  to reflect the current controls. Finally, the current
 *  defaults will be written back to the internal variables
 *
 *
 *  Tasks to perform on end of program
 *  Arm
 *  1 - Pincher
 *  2 - Reactor
 *  3 - WidowX
 *  4 - Snapper
 *
 *  Mode
 *   1 - Cartesian
 *   2 - Cylindrical
 *   3 - backhoe
 *
 *  Wrist Orientaiton
 *    1- Straight
 *    2 - 90 degrees
 ******************************************************/

public void setPositionParameters()
{

  armParam0X = new int[][]{pincherNormalX,reactorNormalX,widowNormalX,widowNormalX, snapperNormalX};
  armParam0Y = new int[][]{pincherNormalY,reactorNormalY,widowNormalY,widowNormalY, snapperNormalY};
  armParam0Z = new int[][]{pincherNormalZ,reactorNormalZ,widowNormalZ,widowNormalZ, snapperNormalZ};
  armParam0WristAngle = new int[][]{pincherNormalWristAngle,reactorNormalWristAngle,widowNormalWristAngle, widowNormalWristAngle, snapperNormalWristAngle};

  armParam90X = new int[][]{pincher90X,reactor90X,widow90X, widow90X, snapper90X};
  armParam90Y = new int[][]{pincher90Y,reactor90Y,widow90Y,widow90Y, snapper90Y};
  armParam90Z = new int[][]{pincher90Z,reactor90Z,widow90Z,widow90Z, snapper90Z};
  armParam90WristAngle = new int[][]{pincher90WristAngle,reactor90WristAngle,widow90WristAngle,widow90WristAngle, snapper90WristAngle};

  armParamBase = new int[][]{pincherBase,reactorBase,widowBase,widowBase, snapperBase};
  armParamBHShoulder = new int[][]{pincherBHShoulder,reactorBHShoulder,widowBHShoulder,widowBHShoulder, snapperBHShoulder};
  armParamBHElbow = new int[][]{pincherBHElbow,reactorBHElbow,widowBHElbow,widowBHElbow, snapperBHElbow};
  armParamBHWristAngle = new int[][]{pincherBHWristAngle,reactorBHWristAngle,widowBHWristAngle,widowBHWristAngle, snapperBHWristAngle};
  armParamBHWristRot = new int[][]{pincherBHWristRot,reactorBHWristRot,widowBHWristRot, widowBHWristRot, snapperBHWristRot};

  armParam0WristRotate = new int[][]{pincherWristRotate,reactorWristRotate,widowWristRotate,widowWristRotate, snapperWristRotate};
  armParamGripper = new int[][]{pincherGripper,reactorGripper,widowGripper,widowGripper, snapperGripper};

  armParamWristAngle0Knob = new int[][]{pincherBHWristAngleNormalKnob,reactorWristAngleNormalKnob,widowBHWristAngleNormalKnob,widowBHWristAngleNormalKnob,snapperBHWristAngleNormalKnob};
  armParamWristAngle90Knob = new int[][]{pincherBHWristAngle90Knob,reactorWristAngle90Knob,widowBHWristAngle90Knob,widowBHWristAngle90Knob, snapperBHWristAngle90Knob};
  armParamWristAngleBHKnob = new int[][]{pincherWristAngleBHKnob,reactorWristAngleBHKnob,widowWristAngleBHKnob,widowWristAngleBHKnob, snapperWristAngleBHKnob};
  armParamWristRotKnob = new int[][]{pincherWristRotKnob,reactorWristRotKnob,widowWristRotKnob,widowWristRotKnob, snapperWristRotKnob};
  armParamBaseKnob = new int[][]{pincherBaseKnob,reactorBaseKnob,widowBaseKnob,widowBaseKnob, snapperBaseKnob};
  armParamShoulderKnob = new int[][]{pincherShoulderKnob,reactorShoulderKnob,widowShoulderKnob,widowShoulderKnob,snapperShoulderKnob};
  armParamElbowKnob = new int[][]{pincherElbowKnob,reactorElbowKnob,widowElbowKnob,widowElbowKnob,snapperElbowKnob};
  armParamElbowKnobRotation = new float[]{pincherElbowKnobRotation,reactorElbowKnobRotation,widowElbowKnobRotation,widowElbowKnobRotation,snapperElbowKnobRotation};





  //armParamDelta new int[][]{pincherElbowKnob,reactorElbowKnob,widowElbowKnob,widowElbowKnob,snapperElbowKnob};


  switch(currentMode)
  {
    //cartesian
    case 1:

      //hide/show appropriate GUI elements
      baseKnob.setVisible(false);
      shoulderKnob.setVisible(false);
      elbowKnob.setVisible(false);
      xSlider.setVisible(true);
      ySlider.setVisible(true);
      zSlider.setVisible(true);
      wristRotateKnob.setVisible(true);

      switch(currentOrientation)
      {
        //straight
        case 1:

        xSlider.setLimits( armParam0X[currentArm-1][0], armParam0X[currentArm-1][1], armParam0X[currentArm-1][2]);
        xTextField.setText(Integer.toString(armParam0X[currentArm-1][0]));
        xLabel.setText("X Coord");
        arrayCopy(armParam0X[currentArm-1], xParameters);

        ySlider.setLimits( armParam0Y[currentArm-1][0], armParam0Y[currentArm-1][1], armParam0Y[currentArm-1][2]) ;
        yTextField.setText(Integer.toString(armParam0Y[currentArm-1][0]));
        yLabel.setText("Y Coord");
        arrayCopy(armParam0Y[currentArm-1], yParameters);

        zSlider.setLimits( armParam0Z[currentArm-1][0], armParam0Z[currentArm-1][1], armParam0Z[currentArm-1][2]) ;
        zTextField.setText(Integer.toString(armParam0Z[currentArm-1][0]));
        zLabel.setText("Z Coord");
        arrayCopy(armParam0Z[currentArm-1], zParameters);


        wristAngleKnob.setTurnRange(armParamWristAngle0Knob[currentArm-1][0], armParamWristAngle0Knob[currentArm-1][1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParam0WristAngle[currentArm-1][0], armParam0WristAngle[currentArm-1][1], armParam0WristAngle[currentArm-1][2]);//set value limits
        wristAngleTextField.setText(Integer.toString(armParam0WristAngle[currentArm-1][0]));
        wristAngleLabel.setText("Wrist Angle");
        arrayCopy(armParam0WristAngle[currentArm-1], wristAngleParameters);


        wristRotateKnob.setTurnRange(armParamWristRotKnob[currentArm-1][0], armParamWristRotKnob[currentArm-1][1]); //set angle limits start/finish
        wristRotateKnob.setLimits(armParam0WristRotate[currentArm-1][0], armParam0WristRotate[currentArm-1][1], armParam0WristRotate[currentArm-1][2]);//set value limits
        wristRotateTextField.setText(Integer.toString(armParam0WristRotate[currentArm-1][0]));
        wristRotateLabel.setText("Wrist Rotate");
        arrayCopy(armParam0WristRotate[currentArm-1], wristRotateParameters);


        gripperSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        gripperTextField.setText(Integer.toString(armParamGripper[currentArm-1][0]));
        gripperLabel.setText("Gripper");
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);

           gripperLeftSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][2], armParamGripper[currentArm-1][1]);
        gripperRightSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);



        break;

        //90 degrees
        case 2:




        xSlider.setLimits( armParam90X[currentArm-1][0], armParam90X[currentArm-1][1], armParam90X[currentArm-1][2]);
        xTextField.setText(Integer.toString(armParam90X[currentArm-1][0]));
        xLabel.setText("X Coord");
        arrayCopy(armParam90X[currentArm-1], xParameters);

        ySlider.setLimits( armParam90Y[currentArm-1][0], armParam90Y[currentArm-1][1], armParam90Y[currentArm-1][2]) ;
        yTextField.setText(Integer.toString(armParam90Y[currentArm-1][0]));
        yLabel.setText("Y Coord");
        arrayCopy(armParam90Y[currentArm-1], yParameters);

        zSlider.setLimits( armParam90Z[currentArm-1][0], armParam90Z[currentArm-1][1], armParam90Z[currentArm-1][2]) ;
        zTextField.setText(Integer.toString(armParam90Z[currentArm-1][0]));
        zLabel.setText("Z Coord");
        arrayCopy(armParam90Z[currentArm-1], zParameters);

        wristAngleKnob.setTurnRange(armParamWristAngle90Knob[currentArm-1][0], armParamWristAngle90Knob[currentArm-1][1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParam90WristAngle[currentArm-1][0], armParam90WristAngle[currentArm-1][1], armParam90WristAngle[currentArm-1][2]);//set value limits
        wristAngleTextField.setText(Integer.toString(armParam90WristAngle[currentArm-1][0]));
        wristAngleLabel.setText("Wrist Angle");
        arrayCopy(armParam90WristAngle[currentArm-1], wristAngleParameters);


        wristRotateKnob.setTurnRange(armParamWristRotKnob[currentArm-1][0], armParamWristRotKnob[currentArm-1][1]); //set angle limits start/finish
        wristRotateKnob.setLimits(armParam0WristRotate[currentArm-1][0], armParam0WristRotate[currentArm-1][1], armParam0WristRotate[currentArm-1][2]);//set value limits
        wristRotateTextField.setText(Integer.toString(armParam0WristRotate[currentArm-1][0]));
        wristRotateLabel.setText("Wrist Rotate");
        arrayCopy(armParam0WristRotate[currentArm-1], wristRotateParameters);


        gripperSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        gripperTextField.setText(Integer.toString(armParamGripper[currentArm-1][0]));
        gripperLabel.setText("Gripper");
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);
       gripperLeftSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][2], armParamGripper[currentArm-1][1]);
        gripperRightSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);

        break;
      }
    break;

      //cylindrical
    case 2:

      //hide/show appropriate GUI elements
      baseKnob.setVisible(true);
      shoulderKnob.setVisible(false);
      elbowKnob.setVisible(false);
      xSlider.setVisible(false);
      ySlider.setVisible(true);
      zSlider.setVisible(true);
      wristRotateKnob.setVisible(true);

      switch(currentOrientation)
      {

        //straight
        case 1:


        wristAngleKnob.setTurnRange(armParamWristAngle0Knob[currentArm-1][0], armParamWristAngle0Knob[currentArm-1][1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParam0WristAngle[currentArm-1][0], armParam0WristAngle[currentArm-1][1], armParam0WristAngle[currentArm-1][2]);//set value limits


        baseKnob.setTurnRange(armParamBaseKnob[currentArm-1][0], armParamBaseKnob[currentArm-1][1]); //set angle limits start/finish
        baseKnob.setLimits(armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);//set value limits


       // baseKnob.setRotation(HALF_PI);
        //xSlider.setLimits( armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);
        xTextField.setText(Integer.toString(armParamBase[currentArm-1][0]));
        xLabel.setText("Base");
        arrayCopy(armParamBase[currentArm-1], xParameters);




        xSlider.setLimits( armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);

        ySlider.setLimits( armParam0Y[currentArm-1][0], armParam0Y[currentArm-1][1], armParam0Y[currentArm-1][2]) ;
        yTextField.setText(Integer.toString(armParam0Y[currentArm-1][0]));
        yLabel.setText("Y Coord");
        arrayCopy(armParam0Y[currentArm-1], yParameters);


        zSlider.setLimits( armParam0Z[currentArm-1][0], armParam0Z[currentArm-1][1], armParam0Z[currentArm-1][2]) ;
        zTextField.setText(Integer.toString(armParam0Z[currentArm-1][0]));
        zLabel.setText("Z Coord");
        arrayCopy(armParam0Z[currentArm-1], zParameters);



        wristAngleKnob.setTurnRange(armParamWristAngle0Knob[currentArm-1][0], armParamWristAngle0Knob[currentArm-1][1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParam0WristAngle[currentArm-1][0], armParam0WristAngle[currentArm-1][1], armParam0WristAngle[currentArm-1][2]);//set value limits
        wristAngleTextField.setText(Integer.toString(armParam0WristAngle[currentArm-1][0]));
        wristAngleLabel.setText("Wrist Angle");
        arrayCopy(armParam0WristAngle[currentArm-1], wristAngleParameters);

        wristRotateKnob.setTurnRange(armParamWristRotKnob[currentArm-1][0], armParamWristRotKnob[currentArm-1][1]); //set angle limits start/finish
        wristRotateKnob.setLimits(armParam0WristRotate[currentArm-1][0], armParam0WristRotate[currentArm-1][1], armParam0WristRotate[currentArm-1][2]);//set value limits
        wristRotateTextField.setText(Integer.toString(armParam0WristRotate[currentArm-1][0]));
        wristRotateLabel.setText("Wrist Rotate");
        arrayCopy(armParam0WristRotate[currentArm-1], wristRotateParameters);


        gripperSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        gripperTextField.setText(Integer.toString(armParamGripper[currentArm-1][0]));
        gripperLabel.setText("Gripper");
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);
        gripperLeftSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][2], armParamGripper[currentArm-1][1]);
        gripperRightSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);

        break;

        //90 degrees
        case 2:

        wristAngleKnob.setTurnRange(armParamWristAngle90Knob[currentArm-1][0], armParamWristAngle90Knob[currentArm-1][1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParam90WristAngle[currentArm-1][0], armParam90WristAngle[currentArm-1][1], armParam90WristAngle[currentArm-1][2]);//set value limits

        xTextField.setText(Integer.toString(armParamBase[currentArm-1][0]));
        xLabel.setText("Base");
        arrayCopy(armParamBase[currentArm-1], xParameters);

        ySlider.setLimits( armParam90Y[currentArm-1][0], armParam90Y[currentArm-1][1], armParam90Y[currentArm-1][2]) ;
        yTextField.setText(Integer.toString(armParam90Y
        [currentArm-1][0]));
        yLabel.setText("Y Coord");
        arrayCopy(armParam90Y[currentArm-1], yParameters);


        zSlider.setLimits( armParam90Z[currentArm-1][0], armParam90Z[currentArm-1][1], armParam90Z[currentArm-1][2]) ;
        zTextField.setText(Integer.toString(armParam90Z[currentArm-1][0]));
        zLabel.setText("Z Coord");
        arrayCopy(armParam90Z[currentArm-1], zParameters);





        xSlider.setLimits( armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);
        xTextField.setText(Integer.toString(armParamBase[currentArm-1][0]));
        xLabel.setText("X Coord");
        arrayCopy(armParamBase[currentArm-1], xParameters);


        ySlider.setLimits( armParam90Y[currentArm-1][0], armParam90Y[currentArm-1][1], armParam90Y[currentArm-1][2]) ;
        yTextField.setText(Integer.toString(armParam90Y[currentArm-1][0]));
        yLabel.setText("Y Coord");
        arrayCopy(armParam90Y[currentArm-1], yParameters);

        zSlider.setLimits( armParam90Z[currentArm-1][0], armParam90Z[currentArm-1][1], armParam90Z[currentArm-1][2]) ;
        zTextField.setText(Integer.toString(armParam90Z[currentArm-1][0]));
        zLabel.setText("Z Coord");
        arrayCopy(armParam90Z[currentArm-1], zParameters);

        wristAngleKnob.setTurnRange(armParamWristAngle90Knob[currentArm-1][0], armParamWristAngle90Knob[currentArm-1][1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParam90WristAngle[currentArm-1][0], armParam90WristAngle[currentArm-1][1], armParam90WristAngle[currentArm-1][2]);//set value limits
        wristAngleTextField.setText(Integer.toString(armParam90WristAngle[currentArm-1][0]));
        wristAngleLabel.setText("Wrist Angle");
        arrayCopy(armParam90WristAngle[currentArm-1], wristAngleParameters);


        wristRotateKnob.setTurnRange(armParamWristRotKnob[currentArm-1][0], armParamWristRotKnob[currentArm-1][1]); //set angle limits start/finish
        wristRotateKnob.setLimits(armParam0WristRotate[currentArm-1][0], armParam0WristRotate[currentArm-1][1], armParam0WristRotate[currentArm-1][2]);//set value limits
        wristRotateTextField.setText(Integer.toString(armParam0WristRotate[currentArm-1][0]));
        wristRotateLabel.setText("Wrist Rotate");
        arrayCopy(armParam0WristRotate[currentArm-1], wristRotateParameters);


        gripperSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        gripperTextField.setText(Integer.toString(armParamGripper[currentArm-1][0]));
        gripperLabel.setText("Gripper");
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);
       gripperLeftSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][2], armParamGripper[currentArm-1][1]);
        gripperRightSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);


        break;
      }

      break;

      //backhoe
      case 3:

        baseKnob.setVisible(true);
        shoulderKnob.setVisible(true);
        elbowKnob.setVisible(true);
        xSlider.setVisible(false);
        ySlider.setVisible(false);
        zSlider.setVisible(false);
        wristRotateKnob.setVisible(true);




        baseKnob.setTurnRange(armParamBaseKnob[currentArm-1][0], armParamBaseKnob[currentArm-1][1]); //set angle limits start/finish
        baseKnob.setLimits(armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);//set value limits
        xTextField.setText(Integer.toString(armParamBase[currentArm-1][0]));
        xLabel.setText("Base");
        arrayCopy(armParamBase[currentArm-1], xParameters);


        shoulderKnob.setTurnRange(armParamShoulderKnob[currentArm-1][0], armParamShoulderKnob[currentArm-1][1]); //set angle limits start/finish
        shoulderKnob.setLimits(armParamBHShoulder[currentArm-1][0], armParamBHShoulder[currentArm-1][1], armParamBHShoulder[currentArm-1][2]);//set value limits
        yTextField.setText(Integer.toString(armParamBHShoulder[currentArm-1][0]));
        yLabel.setText("Shoulder");
        arrayCopy(armParamBHShoulder[currentArm-1], yParameters);


        elbowKnob.setTurnRange(armParamElbowKnob[currentArm-1][0], armParamElbowKnob[currentArm-1][1]); //set angle limits start/finish
        elbowKnob.setLimits(armParamBHElbow[currentArm-1][0], armParamBHElbow[currentArm-1][1], armParamBHElbow[currentArm-1][2]);//set value limits
        elbowKnob.setRotation(armParamElbowKnobRotation[currentArm-1],GControlMode.CENTER);



        zTextField.setText(Integer.toString(armParamBHElbow[currentArm-1][0]));
        zLabel.setText("Elbow");
        arrayCopy(armParamBHElbow[currentArm-1], zParameters);

        wristAngleSlider.setLimits(armParamBHWristAngle[currentArm-1][0], armParamBHWristAngle[currentArm-1][1], armParamBHWristAngle[currentArm-1][2]);
        wristAngleTextField.setText(Integer.toString(armParamBHWristAngle[currentArm-1][0]));
        wristAngleLabel.setText("Wrist Angle");
        arrayCopy(armParamBHWristAngle[currentArm-1], wristAngleParameters);

        wristRotateTextField.setText(Integer.toString(armParamBHWristRot[currentArm-1][0]));
        wristRotateLabel.setText("Wrist Rotate");
        arrayCopy(armParamBHWristRot[currentArm-1], wristRotateParameters);


        gripperSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        gripperTextField.setText(Integer.toString(armParamGripper[currentArm-1][0]));
        gripperLabel.setText("Gripper");
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);



        gripperLeftSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][2], armParamGripper[currentArm-1][1]);
        gripperRightSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);


        wristAngleKnob.setTurnRange(reactorWristAngleBHKnob[0], reactorWristAngleBHKnob[1]); //set angle limits start/finish
        wristAngleKnob.setLimits(armParamBHWristAngle[currentArm-1][0], armParamBHWristAngle[currentArm-1][1], armParamBHWristAngle[currentArm-1][2]);//set value limits
        arrayCopy(armParamBHWristAngle[currentArm-1], wristAngleParameters);


        wristRotateKnob.setTurnRange(reactorWristRotKnob[0], reactorWristRotKnob[1]); //set angle limits start/finish
        wristRotateKnob.setLimits(armParamBHWristRot[currentArm-1][0], armParamBHWristRot[currentArm-1][1], armParamBHWristRot[currentArm-1][2]);//set value limits


        wristRotateTextField.setText(Integer.toString(armParamBHWristRot[currentArm-1][0]));
        wristRotateLabel.setText("Wrist Rotate");
        arrayCopy(armParamBHWristRot[currentArm-1], wristRotateParameters);

        wristRotateSlider.setVisible(false);
        wristRotateTextField.setVisible(true);
        wristRotateLabel.setVisible(true);


        gripperSlider.setLimits( armParamGripper[currentArm-1][0], armParamGripper[currentArm-1][1], armParamGripper[currentArm-1][2]);
        gripperTextField.setText(Integer.toString(armParamGripper[currentArm-1][0]));
        gripperLabel.setText("Gripper");
        arrayCopy(armParamGripper[currentArm-1], gripperParameters);




          xSlider.setLimits( armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);
          ySlider.setLimits( armParamBHShoulder[currentArm-1][0], armParamBHShoulder[currentArm-1][1], armParamBHShoulder[currentArm-1][2]);
          zSlider.setLimits( armParamBHElbow[currentArm-1][0], armParamBHElbow[currentArm-1][1], armParamBHElbow[currentArm-1][2]);

//        xSlider.setLimits( armParamBase[currentArm-1][0], armParamBase[currentArm-1][1], armParamBase[currentArm-1][2]);
//
//
//        ySlider.setLimits( armParamShoulder[currentArm-1][0], armParamShoulderKnob[currentArm-1][1], armParamShoulderKnob[currentArm-1][2]) ;
//
//
//        zSlider.setLimits( armParamElbowKnob[currentArm-1][0], armParamElbowKnob[currentArm-1][1], armParamElbowKnob[currentArm-1][2]) ;
//





        break;
      }




  //show or hide wrist rotate -if all the WR parameters are 0 then hide wrist angle
  if(armParam0WristRotate[currentArm-1][0] == 0 && armParam0WristRotate[currentArm-1][1] == 0 && armParam0WristRotate[currentArm-1][2] == 0)
  {
    wristRotateTextField.setVisible(false);
    wristRotateKnob.setVisible(false);
    wristRotateLabel.setVisible(false);
  }
  else
  {

    wristRotateTextField.setVisible(true);
    wristRotateKnob.setVisible(true);
    wristRotateLabel.setVisible(true);

  }

  //reset deltas

  deltaTextField.setText("125");
  deltaTextField.setLocalColorScheme(GCScheme.BLUE_SCHEME);
  deltaTextField.setOpaque(true);
  deltaTextField.addEventHandler(this, "deltaTextField_change");



  deltaSlider.setShowValue(true);
  deltaSlider.setShowLimits(true);
  deltaSlider.setLimits(125.0f, 0.0f, 255.0f);
  deltaSlider.setEasing(0.0f);
  deltaSlider.setNumberFormat(G4P.INTEGER, 0);
  deltaSlider.setLocalColorScheme(GCScheme.BLUE_SCHEME);
  deltaSlider.setOpaque(false);
  deltaSlider.addEventHandler(this, "deltaSlider_change");



  deltaLabel.setTextAlign(GAlign.LEFT, GAlign.MIDDLE);
  deltaLabel.setText("Delta");
  deltaLabel.setLocalColorScheme(GCScheme.BLUE_SCHEME);
  deltaLabel.setOpaque(false);




  if(currentArm == 5)
  {
    modePanel.setVisible(false);
    wristPanel.setVisible(false);
    emergencyStopButton.setVisible(false);

  digitalCheckbox1.setText("2");
  digitalCheckbox2.setText("4");
  digitalCheckbox3.setText("7");
  digitalCheckbox4.setText("8");
  digitalCheckbox5.setText("11");
  digitalCheckbox6.setText("12");
  digitalCheckbox7.setText("13");

//  digitalCheckbox5.moveTo(154,35);
//  digitalCheckbox6.moveTo(192,35);
//  digitalCheckbox7.moveTo(230,35);
  }

  else
  {
    modePanel.setVisible(true);
    wristPanel.setVisible(true);


  digitalCheckbox1.setText("1");
  digitalCheckbox2.setText("2");
  digitalCheckbox3.setText("3");
  digitalCheckbox4.setText("4");
  digitalCheckbox5.setText("5");
  digitalCheckbox6.setText("6");
  digitalCheckbox7.setText("7");

//  digitalCheckbox5.moveTo(144,35);
//  digitalCheckbox6.moveTo(172,53);
//  digitalCheckbox7.moveTo(200,35);
//
  }






















    //write defualt parameters back to internal values
    xCurrent = xParameters[0];
    yCurrent = yParameters[0];
    zCurrent = zParameters[0];
    wristAngleCurrent = wristAngleParameters[0];
    wristRotateCurrent = wristRotateParameters[0];
    gripperCurrent = gripperParameters[0];
    deltaCurrent = deltaParameters[0];
    extendedTextField.setText("0");
    updateOffsetCoordinates();

println(xCurrent);
println(xParameters[0]);
println(yCurrent);
println(zCurrent);
println(wristAngleCurrent);
println(wristRotateCurrent);
println(gripperCurrent);
}//end set postiion parameters
/***********************************************************************************
 *  }--\     InterbotiX     /--{
 *      |    Arm Link      |
 *   __/                    \__
 *  |__|                    |__|
 *
 *  importParse.pde
 *
 *  Test importing  functionality
 *
********************************************************************************/

BufferedReader reader;
String line;



public void readArmFile(File selection)
{
  noLoop();
  String[] txtFile;

  if (selection == null)
  {

    loop();
    displayError("No File Selected.","");
    return;
  }



  String filepath = selection.getAbsolutePath();
  println("User selected " + filepath);







        try
      {
        txtFile = loadStrings(filepath);
      }
      catch (Exception e)
      {
        txtFile = null;
        loop();
        displayError("Problem With File.","");
        return;
      }


// load file here





  String armNumberString;
  int armNumberInt;
  String sequenceNumberString;
  int sequenceNumberInt;
  String armModeString;
  int armModeInt;
  String armOrientationString;
  int armOrientationInt;
  String tempLine ;
  String[] splitPose = {"","","","","","","",""};
  int[] tempPoseData = {0, 0, 0, 0,0 ,0 ,0 ,0,0};
  int[] tempPoseData2 = {0, 0, 0, 0,0 ,0 ,0 ,0,0};
  int[] tempPoseData5=  {0, 0, 0, 0,0 ,0 ,0 ,0,0};




    if(txtFile[0].length() >= 6)
    {
      armNumberString = txtFile[0].substring(6, txtFile[0].length());
      armNumberInt = Integer.parseInt(armNumberString);

    }
    else
    {

      loop();
      displayError("Problem With File.","");
      return;

    }
    //println(armNumberInt);

    //check if this file works for the arm currently connected
    if(armNumberInt != currentArm)
    {
      loop();
      printlnDebug("Wrong Arm");
      displayError("Incorrect File - File for the wrong Arm or the wrong type of file","");
      return;
    }


   clearPoses();


    sequenceNumberString = txtFile[1].substring(11, txtFile[1].length());
    sequenceNumberInt = Integer.parseInt(sequenceNumberString);



    armModeString = txtFile[2].substring(7, txtFile[2].length());
    armModeInt = Integer.parseInt(armModeString);



    if(armModeInt == 1)
    {

      setCartesian();
    }
    else if(armModeInt == 2)
    {
      setCylindrical();
    }
    else if(armModeInt == 3)
    {
      setBackhoe();
    }





    armOrientationString = txtFile[3].substring(14, txtFile[3].length());
    armOrientationInt = Integer.parseInt(armOrientationString);
    //println(armOrientationString);


    if (armOrientationInt == 1)
    {
      setOrientStraight();

    }
    else if(armOrientationInt == 2)
    {
      setOrient90();
    }


    //read the posotion data from the file.
    for(int j = 0; j < sequenceNumberInt ;j++)
    {



      tempLine = txtFile[22+(j*6)].replace("    IKSequencingControl(", "");
      tempLine = tempLine.replace(", playState);", "");


      splitPose = tempLine.split(",");


     for(int i = 0;i < splitPose.length;i++)
      {





        splitPose[i] = splitPose[i].replaceAll("\\s","") ;
        tempPoseData[i] = Integer.parseInt(splitPose[i]);

      }


   switch(armModeInt)
    {
       case 1:
         //x, wrist angle, and wrist rotate must be offset, all others are normal
         tempPoseData2[0] = tempPoseData[0] - 512;
         tempPoseData2[1] = tempPoseData[1];
         tempPoseData2[2] = tempPoseData[2];
         tempPoseData2[3] = tempPoseData[3];
         tempPoseData2[4] = tempPoseData[4];
         tempPoseData2[5] = tempPoseData[5];
         tempPoseData2[6] = tempPoseData[6]/16;
         tempPoseData2[7] = 0;
         tempPoseData2[8] = tempPoseData[7];


         break;

       case 2:

         //wrist angle, and wrist rotate must be offset, all others are normal


         tempPoseData2[0] = tempPoseData[0];




         tempPoseData2[1] = tempPoseData[1];
         tempPoseData2[2] = tempPoseData[2];
         tempPoseData2[3] = tempPoseData[3];
         tempPoseData2[4] = tempPoseData[4];
         tempPoseData2[5] = tempPoseData[5];
         tempPoseData2[6] = tempPoseData[6]/16;
         tempPoseData2[7] = 0;
         tempPoseData2[8] = tempPoseData[7];



         break;

       case 3:

         //no offsets needed
         tempPoseData2[0] = tempPoseData[0];
         tempPoseData2[1] = tempPoseData[1];
         tempPoseData2[2] = tempPoseData[2];
         tempPoseData2[3] = tempPoseData[3];
         tempPoseData2[4] = tempPoseData[4];
         tempPoseData2[5] = tempPoseData[5];
         tempPoseData2[6] = tempPoseData[6];
         tempPoseData2[7] = 0;
         tempPoseData2[8] = tempPoseData[7];
        break;
    }

     //pauseTextField.setText(tempPoseData[7] + "");




          addNewPose(tempPoseData2);





      printDebug("Added Pose ");
      printlnDebug(j + " ");



    }



  // for(int i = 0; i < poses.size(); i++)
  // {

  //   sequencePanel.addControl(poses.get(i));
  // }

   loop();
    return;

}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "ArmLink" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
