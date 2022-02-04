// 2021-2022 FTC Freight Frenzy
//
// Current Version: 2021-11-06 - Kai Rodriguez
//
// turn 45 degrees
// sleep for a second
// drives one meter froward
// sleep for a second
// crab 100 mm to the right
// all at 0.25 power
//
// List Code Inputs here
// List Code Outputs here
//
// List non-standard dependancies here
//
// Version History
// Current Version: Added comments and references to the code 2021-11-05
// Previous Version: Competion Version - Release Date Unknown
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TOFTest", group="Autonomous")
public class TOFTest extends LinearOpMode {
    
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
    private DistanceSensor TOF_Left;// Time Of Flight Left
    private DistanceSensor TOF_Right;// Time Of Flight Right
    private Servo PLBD;              // PreLoaded Block Dropper
    
    String MOTORTYPE = "ANDY20"; // "REV20", "REV40", "REV60", "TETRIX"
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
                WheelDiameterMM = 103.0;
                TickPerDegree = 8.769; // 1.46752 rotations per degree, empiracally derived
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "REV20" :
                TicksPerRot = 560.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752/90;
                DistCrabPerRot = 10; //still to be empiracally derived
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
        TOF_Left = hardwareMap.get(DistanceSensor.class, "TOF_Left");
        TOF_Right = hardwareMap.get(DistanceSensor.class, "TOF_Right");
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
        
        WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
        WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        
        telemetry.addData("fwd-left connection info", WHEEL_FL.getConnectionInfo());
        telemetry.update();
        
        // Waiting for "Play" button to be pressed on the driver hub
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Right-DistanceCM", TOF_Left.getDistance(DistanceUnit.CM)); 
            telemetry.addData("Left-DistanceCM", TOF_Right.getDistance(DistanceUnit.CM)); 
            telemetry.update();
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
        
        WHEEL_FR.setTargetPosition(RightRot);
        WHEEL_FL.setTargetPosition(LeftRot);
        WHEEL_BR.setTargetPosition(RightRot);
        WHEEL_BL.setTargetPosition(LeftRot);
        
        RunMotors(speed); 
        
    }  // End turn
    
    
    private void crab(double distance, double speed) {
        
        int ROT = (int) Math.round( (distance / DistCrabPerRot) * TicksPerRot );
        
        WHEEL_FR.setTargetPosition(-ROT);
        WHEEL_FL.setTargetPosition(ROT);
        WHEEL_BR.setTargetPosition(ROT);
        WHEEL_BL.setTargetPosition(-ROT);
        
        RunMotors(speed);   
        
    } // End crab
    
    
    private void drive(double distance, double speed) {
        
        int ROT = (int) Math.round( (distance / DistPerRot) * TicksPerRot );
        
        WHEEL_FR.setTargetPosition(ROT);
        WHEEL_FL.setTargetPosition(ROT);
        WHEEL_BR.setTargetPosition(ROT);
        WHEEL_BL.setTargetPosition(ROT);
        
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