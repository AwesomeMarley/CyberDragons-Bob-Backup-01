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
// 2021-11-16: Added code Open/Close the Grabber - NOTE "PLBD" in config file on robot is now "Grabber"
//
// Previous Version:
// 2021-11-15: Added code to move the ARM
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

@Autonomous(name = "LSO3", group="Autonomous")
public class LSO3 extends LinearOpMode { //Locate Special Object

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
    private DistanceSensor TOF_Back; // Time Of Flight Back
    private Servo Grabber;           // Grabber

    String MOTORTYPE = "REV20"; // "REV20", "REV40", "REV60", "TETRIX"
    String ARMMOTOR = "TETRIX"; // "REV20", "REV40", "REV60", "TETRIX"

    int[] ArmLevelTicks = {0, 0, 0, 0};  // declares an array of integers for arm level heights
    // ArmLevelTicks[Level] -> Ticks to move the arm to
    // ArmLevelTicks[0] = Arm Rest Position after Initialization
    // ArmLevelTicks[Level] = Birdbath Levels

    // That is which bar code corresponds to which birdbath level, it switches depending on which side ofthe bird bath you are on
    int StartPosition = 4;         // 1 = Red Closest to duck spinner, 2 = Red Closest to Warehouse; 3 = Blue Closest to duck spinner, 4 = Blue Closest to Warehouse

    double WheelTicksPerRot; // DEFAULT to ANDYMARK
    double WheelDiameterMM;         // The Diameter of the drive wheels in mm
    double DistPerRot;              // Distance traveled, either forward or backward, per 1 complete wheel rotation
    double TickPerDegree;           // The number of motor ticks to rotate (tank-like turn) the robot, either clockwise (CW) or counter-clockwise (CCW)
    double DistCrabPerRot;          // Distance crabbed, either left or right, per 1 complete wheel rotation

    int Step = -1;            // Variable to track which step of the of autonomous to perform
    int[] BarCodeLevels = {1, 2, 3}; // An int array for which barcode level corresponds to which level on the birdbath
    int Level = BarCodeLevels[0];   // Need to update when updating start position that is if the BarCodeLevels is reversed based on starting position
    //int StartLevel = 2;
    boolean ObjDetected = false;    // Boolean, for if there Special Object has been detected
    double BC_Dist = 55.0;          // Bar Code Distance
    double TOFcm = 0.0;             // Time of Flight Measurement in cm

    int WHEEL_FR_pos = 0;           // Front Right Wheel Position in Encoder Steps
    int WHEEL_FL_pos = 0;           // Front Left Wheel Position in Encoder Steps
    int WHEEL_BR_pos = 0;           // Back Right Wheel Position in Encoder Steps
    int WHEEL_BL_pos = 0;           // Back Left Wheel Position in Encoder Steps

    double ArmTicksPerRot; // DEFAULT to ANDYMARK
    double ArmDownSpeed = 0.35;      // Speed to move the arm down 0.0<->1.0
    double ArmUpSpeed = 1.00;     // Speed to move the arm down 0.0<->1.0
    double ArmHoldSpeed = 0.1;      // Holding Current For the Arm
    double OpenGrabberPos = 0.35;
    double ClosedGrabberPos = 0.0;
    boolean GrabberOpen = false;
    double ARM_pos;
    double closed = 0.0, open = 0.35;
    long ToTheSide;
    int TurnAmount;

    @Override
    public void runOpMode() {

        switch (MOTORTYPE) {
            case "ANDY20":
                WheelTicksPerRot = 537.6;
                WheelDiameterMM = 103.5;
                TickPerDegree = 8.769; //
                DistCrabPerRot = 267.5; //mm
                break;
            case "REV20":
                WheelTicksPerRot = 430.77;
                WheelDiameterMM = 105.0;
                TickPerDegree = 6.69;
                DistCrabPerRot = 260; //still to be empiracally derived
                break;
            case "REV40":
                WheelTicksPerRot = 1120.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "REV60":
                WheelTicksPerRot = 1680.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "TETRIX":
                WheelTicksPerRot = 1440;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            default:
                telemetry.addData("Wheel Motors not specified", "defaulting to ANDY20");
                telemetry.update();

                WheelTicksPerRot = 537.6;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empirically derived
                break;
        } // switch(MOTORTYPE)

        switch (ARMMOTOR) {

            case "ANDY20":
                ArmTicksPerRot = 537.6;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV20":
                ArmTicksPerRot = 1680.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV40":
                ArmTicksPerRot = 1120.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV60":
                ArmTicksPerRot = 1680.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "TETRIX":
                ArmTicksPerRot = 1440;
                ArmLevelTicks = new int[]{-50, -250, -500, -770}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            default:
                telemetry.addData("Arm Motor not specified", "defaulting to ANDY20");
                telemetry.update();

                ArmTicksPerRot = 537.6;
                break;
        } // switch(ARMMOTOR)

        switch (StartPosition) {
            case 1: // Red closest to the duck spinner
                //Collections.reverse(Arrays.asList(BarCodeLevels));
                ToTheSide = 610;
                TurnAmount = -90;
                break;
            case 2: // Red closest to the warehouse
                BarCodeLevels[0]=3;
                BarCodeLevels[2]=1;
                Level=BarCodeLevels[0];
                ToTheSide = -610;
                TurnAmount = -90;
                break;
            case 3: // Blue closest to the duck spinner
                ToTheSide = 610;
                TurnAmount = 90;
                break;
            case 4: // Blue closest to the warehouse
                //Collections.reverse(Arrays.asList(BarCodeLevels));
                BarCodeLevels[0]=3;
                BarCodeLevels[2]=1;
                Level=BarCodeLevels[0];
                ToTheSide = -530;
                TurnAmount = 90;
                break;
            default:
                telemetry.addData("Unspecified StartPosition", "defaulting to 1"); // Red closest to the duck spinner
                telemetry.update();
                //Collections.reverse(Arrays.asList(BarCodeLevels));
                break;
        } // switch(StartPosition)

        DistPerRot = Math.PI * WheelDiameterMM; // Distance traveled per full wheel rotation in mm


        WHEEL_FL = hardwareMap.get(DcMotor.class, "wheelfl");           // Front Left
        WHEEL_FR = hardwareMap.get(DcMotor.class, "wheelfr");           // Front Right
        WHEEL_BL = hardwareMap.get(DcMotor.class, "wheelbl");           // Back Left
        WHEEL_BR = hardwareMap.get(DcMotor.class, "wheelbr");           // Back Right
        TTS_Motor = hardwareMap.get(DcMotor.class, "duck");             // Motor for Duck Turn Table Spinner
        ARM = hardwareMap.get(DcMotor.class, "Arm");                // Motor for raising and lowering the arm
        TOF_Back = hardwareMap.get(DistanceSensor.class, "TOF_Back");   // Time of Flight Sensor mounted on the back of the robot
        Grabber = hardwareMap.get(Servo.class, "Grabber");              // Grabber Servo

        //Reseting encoder value
        WHEEL_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TTS_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Still need to wire this up
        ARM.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Grabber.setPosition(closed);

        WHEEL_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TTS_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
            WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // (MOTORTYPE == "REV20")
        //TTS_Motor.setDirection(DcMotorSimple.Direction.REVERSE); // Not sure if needed
        ARM.setDirection(DcMotorSimple.Direction.REVERSE);  // Not sure if needed


        // *******************************************************************************
        // Step == -1 //Initialization
        // *******************************************************************************

        Grabber.setPosition(ClosedGrabberPos);
        GrabberOpen = false;
        // RunArm(ArmLevelTicks[Level], ArmUpSpeed);

        // telemetry.addData("fwd-left connection info", WHEEL_FL.getConnectionInfo());
        telemetry.addData("Level: ", 0);
        telemetry.update();
        Step++;

        // *******************************************************************************


        // *******************************************************************************
        // Waiting for "Play" button to be pressed on the driver hub
        // *******************************************************************************

        waitForStart();

        // *******************************************************************************


        // *******************************************************************************
        // Step == 0 // Locate Barcode Object and Crab towards Birdbath
        // *******************************************************************************
        RunArm(ArmLevelTicks[Level], ArmUpSpeed);
        
        TOFcm = TOF_Back.getDistance(DistanceUnit.CM);

        if (TOFcm < BC_Dist) { // If Time of flight sensor is less than 55 cm do this
            Level = BarCodeLevels[1]; // Set level to two
            telemetry.addData("Level: ", Level);
            telemetry.addData("TOF: ", TOFcm);
            telemetry.update();
            ObjDetected = true;
        } //(TOFcm < BC_Dist)

        crab(ToTheSide, 0.25); // 61cm to the right, speed = 25%

        // If the Special Object was not Detected in Posistion 2 or 3, set to position 1.
        // If the object is in 1 of 3 locations, only need to measure 2 in order to determine the location
        if (!ObjDetected) {
            telemetry.addData("Level: ", Level);
            telemetry.addData("TOF: ", TOFcm);
            telemetry.update();
            ObjDetected = true;
        } // (!ObjDetected)
        Step++; // Move to Next Step

        // *******************************************************************************


        // *******************************************************************************
        // Step == 1 // Drive towards Birdbath and move the arm to the correct position
        // *******************************************************************************
        {
            double speed = 0.25;
            if (Level == 1)
                speed = 0.15;
            drive(-570, speed); // 50cm away from the wall, speed = 25%
            Step++;
        }
        // *******************************************************************************


        // *******************************************************************************
        // Step == 2 // At Birdbath - Drop Object
        // *******************************************************************************
        Grabber.setPosition(OpenGrabberPos);
        GrabberOpen = true;
        sleep(500); // Time for the object to fall out
        // *******************************************************************************
        
        drive(100, 0.5);
        turn(TurnAmount, 0.25);
        Step++;
        RunArm(ArmLevelTicks[2], ArmUpSpeed);
        drive(-1500, 1);

        // *******************************************************************************
        // Step == 3 // Drive to Duck Spinner or to Warehouse - Depending on Start Position
        // *******************************************************************************

        // *******************************************************************************


        // *******************************************************************************
        // Step == 4 // Spin Duck Spinner or Grab Freight from Warehouse - Depending on Start Position
        // *******************************************************************************

        // *******************************************************************************


        // *******************************************************************************
        // Step == 5 // Park in Warehouse or designated area  - Depending on Start Position
        // *******************************************************************************

        // *******************************************************************************

    } // End runOpMode

    //******************************
    //********** SubFXNs ***********
    //******************************

    private void turn(double angle, double speed) {

        // int LeftRot = (int) Math.round( angle * TickPerDegree * TicksPerRot);
        // int RightRot = (int) Math.round( -angle * TickPerDegree * TicksPerRot);
        int LeftRot = (int) Math.round(angle * TickPerDegree);
        int RightRot = (int) Math.round(-angle * TickPerDegree);

        WHEEL_FR.setTargetPosition(RightRot + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(LeftRot + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(RightRot + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(LeftRot + WHEEL_BL_pos);

        RunMotors(speed);

    } // End turn


    private void crab(double distance, double speed) {

        int ROT = (int) Math.round((distance / DistCrabPerRot) * WheelTicksPerRot);

        WHEEL_FR.setTargetPosition(-ROT + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(-ROT + WHEEL_BL_pos);

        RunMotors(speed);

    } // End crab


    private void drive(double distance, double speed) {

        int ROT = (int) Math.round((distance / DistPerRot) * WheelTicksPerRot);

        WHEEL_FR.setTargetPosition(ROT + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(ROT + WHEEL_BL_pos);

        RunMotors(speed);

    } // End drive


    private void RunMotors(double speed) {

        WHEEL_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        WHEEL_FL.setPower(speed);
        WHEEL_FR.setPower(speed);
        WHEEL_BL.setPower(speed);
        WHEEL_BR.setPower(speed);

        while (opModeIsActive() && isWheelsMoving()) {
            DoStuffWhileMoving();
        }

        WHEEL_FL.setPower(0.0);
        WHEEL_FR.setPower(0.0);
        WHEEL_BL.setPower(0.0);
        WHEEL_BR.setPower(0.0);

        sleep(100); // Allow for wheels to settle before getting final Position

        WHEEL_FR_pos = WHEEL_FR.getCurrentPosition();
        WHEEL_FL_pos = WHEEL_FL.getCurrentPosition();
        WHEEL_BR_pos = WHEEL_BR.getCurrentPosition();
        WHEEL_BL_pos = WHEEL_BL.getCurrentPosition();

    } // End RunMotors


    // FXN to clean up the code
    private boolean isWheelsMoving() {
        return WHEEL_FL.isBusy() && WHEEL_FR.isBusy() && WHEEL_BL.isBusy() && WHEEL_BR.isBusy();
    } // End wheelIsMoving


    private void RunArm(int ArmTicks, double ArmSpeed) {

        ARM.setTargetPosition(ArmTicks);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARM.setPower(ArmSpeed);

        while (opModeIsActive() && ARM.isBusy()) {idle();} //
        
        ARM.setPower(ArmHoldSpeed);
        sleep(100); // Allow for wheels to settle before getting final Position
        ARM_pos = ARM.getCurrentPosition();

    } // End RunArm

    // Run Arm but without while loop
    private void RunArmAsync(int ArmTicks, double ArmSpeed) {
        if (ARM.isBusy())
            return;

        if (ARM.getCurrentPosition() == ArmTicks) {
            ARM.setPower(ArmHoldSpeed);
            ARM_pos = ARM.getCurrentPosition();
            return;
        }

        ARM.setTargetPosition(ArmTicks);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARM.setPower(ArmSpeed);
    } // End RunArmAsync


    private void DoStuffWhileMoving() {

        TOFcm = TOF_Back.getDistance(DistanceUnit.CM);
        switch (Step) {
            case 0: // Crab towards bird bath and figure out where the bar code object
                if (!ObjDetected) { // if object not detected then run this
                    if (TOFcm < BC_Dist) { // if all of a sudden it sees somethingthan do this
                        Level = BarCodeLevels[2]; // set level to three
                        telemetry.addData("Level:", Level);
                        telemetry.addData("TOF: ", TOFcm);
                        telemetry.update();
                        ObjDetected = true;
                        Step += 1;
                    } else {
                        telemetry.addData("Level: ", 0);
                        telemetry.addData("TOF: ", TOFcm);
                        telemetry.update();
                    }
                } else {
                    telemetry.addData("Level:", Level);
                    telemetry.addData("TOF: ", TOFcm);
                    telemetry.update();
                }
                break;
            case 1: // Drive towards Bird Bath and Move Arm to Correct Level
                // *******************************
                // Insert Code For Moving Arm Here
                // *******************************
                RunArm(ArmLevelTicks[Level],ArmUpSpeed);
                // if (Level > StartLevel)
                //     RunArmAsync(ArmLevelTicks[Level], ArmUpSpeed);
                // else if (Level < StartLevel)
                //     RunArmAsync(ArmLevelTicks[Level], ArmDownSpeed);
                // break;
            case 2:
                // *******************************
                // Insert Code ? here
                // *******************************
            default:
                telemetry.addData("TOF: ", TOFcm);
                telemetry.update();
                break;
        } // end switch case

    } // end DoStuffWhileMoving


} // End DriveTest
