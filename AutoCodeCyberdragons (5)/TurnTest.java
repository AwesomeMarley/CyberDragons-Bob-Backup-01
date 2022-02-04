// 2021-2022 FTC Freight Frenzy
//
// Current Version: 2021-11-08 - Kai Rodriguez
//
// Code to emprically derive the turn distance of a robot per motor tick.  
// Step 1) Select Motor Type Ln 84 and confirm STEP = 1 on Ln 83
// Step 2) Mark Robot Start positon from front wheels with tape
// Step 3) Run the code once - measure resulting angle.
//         This will give an approximate turn distance per single rotation
// Step 4) Update the infomation in the switch-case starting on Ln 100 for the TickPerDegree
// Step 5) Step on Ln 83, STEP = 5.  This will turn ~360 degrees.
// Step 6) Mark the start of the robot, run the code and measure the resulting angle. It should be close to 45 degrees.
// Step 7) Update the TickPerDegree based on new angle.  Rinse and repeat, until exact.
// Step 8) Reset the code - Uncomment Lns 172 - 176, Comment Ln 179
//
// List Code Inputs here
// List Code Outputs here
//
// List non-standard dependancies here
//
// Version History
// Current Version: Added comments and references to the code 2021-11-08
// Previous Version: Initial Build - 2021-11-05
// Previous Previous Version:
// ...
//
// Written By: Kai Rodriguez, 2021-11-06
// For Jams RoboVikings Team 9887
// 2021-2022 Freight Frenzy
// 
// License / Use Terms - GPL
//


package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TurnTest", group="Autonomous")
public class TurnTest extends LinearOpMode {
    
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
    private Servo PLBD; // PreLoaded Block Dropper
    
    int STEP = 5;
    String MOTORTYPE = "REV20"; // "REV20", "REV40", "REV60", "TETRIX"    // todo: write your code here

    double TicksPerRot; // DEFAULT to ANDYMARK
    double WheelDiameterMM;
    double DistPerRot;
    double TickPerDegree;
    double DistCrabPerRot;
    
    
    int WHEEL_FR_pos = 0;
    int WHEEL_FL_pos = 0;
    int WHEEL_BL_pos = 0;
    int WHEEL_BR_pos = 0;
    
    
    @Override
    public void runOpMode() {
        
        switch(MOTORTYPE){
            case "ANDY20" :
                TicksPerRot = 537.6;
                WheelDiameterMM = 103.5;
                // TickPerDegree = 7.9644; //
                TickPerDegree = 7.6;
                DistCrabPerRot = 267.5; //still to be empiracally derived
                break;
            case "REV20" :
                TicksPerRot = 430.77; //430.77
                WheelDiameterMM = 105.0; //105
                TickPerDegree = 5.567; //5.9733
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
                TicksPerRot = 530.0;
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
        }
        
        DistPerRot = Math.PI * WheelDiameterMM; // Distance traveled per full wheel rotation in mm
        
        
        WHEEL_FL = hardwareMap.get(DcMotor.class, "wheelfl"); // Front Left
        WHEEL_FR = hardwareMap.get(DcMotor.class, "wheelfr"); // Front Right
        WHEEL_BL = hardwareMap.get(DcMotor.class, "wheelbl"); // Back Left
        WHEEL_BR = hardwareMap.get(DcMotor.class, "wheelbr"); // Back Right
        TTS_Motor = hardwareMap.get(DcMotor.class, "duck");   // Motor for Duck Turn Table Spinner
        PLBD = hardwareMap.get(Servo.class, "Lid");         // PreLoaded Block Dropper
        //Reseting encoder value
        WHEEL_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        WHEEL_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PLBD.setPosition(0.5);
        
        if(MOTORTYPE == "REV20") {
        WHEEL_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        WHEEL_BL.setDirection(DcMotorSimple.Direction.REVERSE);            
        }
        else {
        WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
        WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        telemetry.addData("fwd-left connection info", WHEEL_FL.getConnectionInfo());
        telemetry.update();
        
        // Waiting for "Play" button to be pressed on the driver hub
        waitForStart();
        
        if (STEP < 5){
            double NumRot = 1;
            int ROT = (int) Math.round( NumRot * TicksPerRot );

            WHEEL_FR.setTargetPosition(-ROT);
            WHEEL_FL.setTargetPosition(ROT);
            WHEEL_BR.setTargetPosition(-ROT);
            WHEEL_BL.setTargetPosition(ROT);
            RunMotors(0.25);
        }
        else{
            //turn(2880, 0.25);        // angle (+ CW), speed (-1 to 1)
            turn(-90,0.25);
            // drive(1000, 0.25) // mm (+ forward), speed (-1 to 1)
            // crab(300, 0.25)  // mm (+ right), speed (-1 to 1)
        }
    
    } // End runOpMode
    
    //******************************
    //********** SubFXNs *********** 
    //******************************
    
    private void turn(double angle, double speed) {
        
        // int LeftRot = (int) Math.round( angle * TickPerDegree * TicksPerRot);
        // int RightRot = (int) Math.round( -angle * TickPerDegree * TicksPerRot);
        int LeftRot = (int) Math.round( angle * TickPerDegree);
        int RightRot = (int) Math.round( -angle * TickPerDegree);
        
        WHEEL_FR.setTargetPosition(RightRot+WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(LeftRot+WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(RightRot+WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(LeftRot+WHEEL_BL_pos);
        
        RunMotors(speed);
        
    }  // End turn
    
    
    private void crab(double distance, double speed) {
        
        int ROT = (int) Math.round( (distance / DistCrabPerRot) * TicksPerRot );
        
        WHEEL_FR.setTargetPosition(-ROT+WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT+WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT+WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(-ROT+WHEEL_BL_pos);
        
        RunMotors(speed);
        
    } // End crab
    
    
    private void drive(double distance, double speed) {
        
        int ROT = (int) Math.round( (distance / DistPerRot) * TicksPerRot );
        
        WHEEL_FR.setTargetPosition(ROT+WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT+WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT+WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(ROT+WHEEL_BL_pos);
        
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
        
        while(opModeIsActive() && WHEEL_FL.isBusy() && WHEEL_FR.isBusy() && WHEEL_BL.isBusy() && WHEEL_BR.isBusy() )  {
            telemetry.addData("encoder-fwd-left", WHEEL_FL.getCurrentPosition() + "  busy=" + WHEEL_FL.isBusy());
            telemetry.addData("encoder-fwd-right", WHEEL_FR.getCurrentPosition() + "  busy=" + WHEEL_FR.isBusy());
            telemetry.addData("encoder-bwd-left", WHEEL_BL.getCurrentPosition() + "  busy=" + WHEEL_BL.isBusy());
            telemetry.addData("encoder-bwd-right", WHEEL_BR.getCurrentPosition() + "  busy=" + WHEEL_BR.isBusy());
            telemetry.update();
            idle();
        }
        
        WHEEL_FL.setPower(0.0);
        WHEEL_FR.setPower(0.0);
        WHEEL_BL.setPower(0.0);
        WHEEL_BR.setPower(0.0);
        
        WHEEL_FR_pos = WHEEL_FR.getCurrentPosition();
        WHEEL_FL_pos = WHEEL_FL.getCurrentPosition();
        WHEEL_BR_pos = WHEEL_BR.getCurrentPosition();
        WHEEL_BL_pos = WHEEL_BL.getCurrentPosition(); 
        
    } // End RunMotors
    
} // End DriveTest
