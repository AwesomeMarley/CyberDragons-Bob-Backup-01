// 2021-2022 FTC Freight Frenzy
// LSO3 - Locate Special Object
// Current Version: 2021-11-14 - Kai Rodriguez, Luca Cipresso, & Teddy Telanoff
//
// This is the code to test out the time of flight sensor
//
// List Code Inputs here
// List Code Outputs here
//
// List non-standard dependancies here
//
// Version History:
// Current Version:
// 2021-11-15: Added code to move the ARM
//
// Previous Version:
// 2021-11-14: Added Step variable to keep track of what step of the automus code the 
//               robot is doing. Cleaned up code.
// 2021-11-13: Moved while loop out of the DoStuffWhileMoving fxn 
// 2021-11-11: Based off LSO written by Kai Rodriguez
//
// Written By: Kai Rodriguez, Luca Cipresso, and Teddy Telanoff
// For Jams RoboVikings Team 9887
// 2021-2022 Freight Frenzy
// 
// License / Use Terms - GPL
//


package org.firstinspires.ftc.robotcontroller.external.samples;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.logging.Level;
import java.util.*;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "GrabberTest", group="Autonomous")
public class GrabberTest extends LinearOpMode { //Locate Special Object
    
    // gamepad1
    // ********
    // left_stick = Front, Back, Turn Left, Turn Right
    // right_stick_x = Crab Left/Right
    // left_bumper =
    // right_bumper =
    // left_trigger = Turn Table Motor (Duck) CCW
    // right_trigger = Turn Table Motor (Duck) CW
    // A = TBD
    // B = TBD
    // X = Preloaded Block Dropper Down
    // Y = Preloaded Block Dropper Up
    //
    // ********
    // gamepad2
    // ********
    // left_stick = TBD
    // right_stick_x = TBD
    // left_bumper = TBD
    // right_bumper = TBD
    // left_trigger = TBD
    // right_trigger = TBD
    // A = TBD
    // B = TBD
    // X = TBD
    // Y = TBD
    //
    
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WHEEL_FL;
    private DcMotor WHEEL_FR;
    private DcMotor WHEEL_BL;
    private DcMotor WHEEL_BR;
    private DcMotor TTS_Motor;
    private DcMotor ARM;
    private Servo Grabber;              // Grabber
    
    String MOTORTYPE = "ANDY20"; // "REV20", "REV40", "REV60", "TETRIX"
    String ARMMOTOR = "TETRIX"; // "REV20", "REV40", "REV60", "TETRIX"
    
    int[] ArmLevelTicks = {0, 0, 0, 0};  // declares an array of integers for arm level heights
                     // ArmLevelTicks[Level] -> Ticks to move the arm to
                     // ArmLevelTicks[0] = Arm Rest Position after Initialization
                     // ArmLevelTicks[Level] = Birdbath Levels
    int[] BarCodeLevels = {1,2,3}; // An int array for which barcode level corresponds to which level on the birdbath
                                   // That is which bar code corresponds to which birdbath level, it switches depending on which side ofthe bird bath you are on
    int StartPosition = 3;         // 1 = Red Closest to duck spinner, 2 = Red Closest to Warehouse; 3 = Blue Closest to duck spinner, 4 = Blue Closest to Warehouse
    
    double TicksPerRot; // DEFAULT to ANDYMARK
    double WheelDiameterMM;         // The Diameter of the drive wheels in mm
    double DistPerRot;              // Distance traveled, either forward or backward, per 1 complete wheel rotation
    double TickPerDegree;           // The number of motor ticks to rotate (tank-like turn) the robot, either clockwise (CW) or counter-clockwise (CCW) 
    double DistCrabPerRot;          // Distance crabbed, either left or right, per 1 complete wheel rotation
    
    int Step = -1;            // Variable to track which step of the of automus to perform
    int Level = BarCodeLevels[0];   // Need to update when updating start position that is if the BarCodeLevels is reversed based on starting position
    boolean ObjDetected = false;    // Boolean, for if thre Speacial Object has been detected
    double BC_Dist = 55.0;          // Bar Code Distance
    double TOFcm = 0.0;             // Time of Flight Measurement in cm
    
    int WHEEL_FR_pos = 0;           // Front Right Wheel Position in Encoder Steps
    int WHEEL_FL_pos = 0;           // Front Left Wheel Position in Encoder Steps
    int WHEEL_BR_pos = 0;           // Back Right Wheel Position in Encoder Steps
    int WHEEL_BL_pos = 0;           // Back Left Wheel Position in Encoder Steps
    
    double ArmDownSpeed = 0.1;      // Speed to move the arm down 0.0<->1.0
    double ArmUpSpeed   = 0.50;     // Speed to move the arm down 0.0<->1.0
    double ArmHoldSpeed = 0.0;      // Holding Current For the Arm
    double[] GrabberPos = {0.0, 0.35};
    boolean GrabberOpen = false;
    double ARM_pos;
    
    @Override
    public void runOpMode() {
        
        switch(MOTORTYPE){
            case "ANDY20" :
                TicksPerRot = 537.6;
                WheelDiameterMM = 103.5;
                TickPerDegree = 8.769; //
                DistCrabPerRot = 267.5; //mm
                break;
            case "REV20" :
                TicksPerRot = 430.77;
                WheelDiameterMM = 105.0;
                TickPerDegree = 6.69;
                DistCrabPerRot = 260; //still to be empiracally derived
                break;
            case "REV40" :
                TicksPerRot = 1120.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752/90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "REV60" :
                TicksPerRot = 1680.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752/90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "TETRIX" :
                TicksPerRot = 1440;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752/90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            default :
                TicksPerRot = 537.6;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752/90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
        } // switch(MOTORTYPE)
        
        switch(ARMMOTOR){

        case "ANDY20" :
                TicksPerRot = 537.6;
                ArmLevelTicks[0] = 100; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
        ArmLevelTicks[1] = 250;
        ArmLevelTicks[2] = 500;
        ArmLevelTicks[3] = 750;                
        break;
            case "REV20" :
                TicksPerRot = 1680.0;
                ArmLevelTicks[0] = 100; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
        ArmLevelTicks[1] = 250;
        ArmLevelTicks[2] = 500;
        ArmLevelTicks[3] = 750;                
        break;
            case "REV40" :
                TicksPerRot = 1120.0;
                ArmLevelTicks[0] = 100; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
        ArmLevelTicks[1] = 250;
        ArmLevelTicks[2] = 500;
        ArmLevelTicks[3] = 750;
                break;
            case "REV60" :
                TicksPerRot = 1680.0;                
        ArmLevelTicks[0] = 100; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
        ArmLevelTicks[1] = 250;
        ArmLevelTicks[2] = 500;
        ArmLevelTicks[3] = 750;
                break;
            case "TETRIX" :
                TicksPerRot = 1440;
                ArmLevelTicks[0] = 100; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
        ArmLevelTicks[1] = 250;
        ArmLevelTicks[2] = 500;
        ArmLevelTicks[3] = 750;
                break;
            default :
                TicksPerRot = 537.6;
                break;
        } // switch(ARMMOTOR)
        
        switch(StartPosition){
                case 1: // Red closest to the duck spinner
            Collections.reverse(Arrays.asList(BarCodeLevels));
                    Level = BarCodeLevels[0]; // Reset Default Level
                    break;
                //case 2: // Red closest to the warehouse
                //    break;
                //case 3: // Blue closest to the duck spinner
                //    break;
                case 4: // Blue closest to the warehouse
                Collections.reverse(Arrays.asList(BarCodeLevels));
                    Level = BarCodeLevels[0]; // Reset Default Level
                    break;
        } // switch(StartPosition)
        
        DistPerRot = Math.PI * WheelDiameterMM; // Distance traveled per full wheel rotation in mm
        


        ARM = hardwareMap.get(DcMotor.class, "Arm");                     // Motor for raising and lowering the arm
        Grabber = hardwareMap.get(Servo.class, "Grabber");                   // PreLoaded Block Dropper
        
        //Reseting encoder value

        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Grabber.setPosition(GrabberPos[0]);
        GrabberOpen = false;
        

        
        
        
        

        //TTS_Motor.setDirection(DcMotorSimple.Direction.REVERSE); // Not sure if needed
        //ARM.setDirection(DcMotorSimple.Direction.REVERSE);  // Not sure if needed
        
        
        // *******************************************************************************
        // Step == -1 //Initialization
        // *******************************************************************************
        

        RunArm(ArmLevelTicks[0], ArmUpSpeed);

        
    // *******************************************************************************
    
        
        // *******************************************************************************
        // Waiting for "Play" button to be pressed on the driver hub
    // *******************************************************************************
        
        waitForStart();

    RunArm(ArmLevelTicks[3], ArmUpSpeed);
        
    Grabber.setPosition(GrabberPos[1]);
     GrabberOpen = true;
    sleep(1500);
        
    Grabber.setPosition(GrabberPos[0]);
    GrabberOpen = false;
    sleep(2000);
    
    

    } // End runOpMode
    
    //******************************
    //********** SubFXNs *********** 
    //******************************
    
    
    
    private void RunArm(int ArmTicks, double ArmSpeed) {
        
    ARM.setTargetPosition(ArmTicks);
        
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        ARM.setPower(ArmSpeed);
        
        while(opModeIsActive() && ARM.isBusy()){idle();} // 
        
        ARM.setPower(ArmHoldSpeed);
        
        sleep(100); // Allow for wheels to settle before getting final Position
        
        ARM_pos = ARM.getCurrentPosition();
        
    } // End RunArm
    
    

    
} // End DriveTest