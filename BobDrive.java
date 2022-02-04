package com.cyberdragons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BobDrive", group="Linear Opmode")

public class BobDrive extends LinearOpMode {
    
    /*
     * *******
     * gamepad1
     * ********
     * left_stick = Front, Back, Turn Left, Turn Right
     * right_stick_x = Crab Left/Right
     * right_bumper = Micro
     * left_trigger = Turn Table Motor (Duck) CCW
     * right_trigger = Turn Table Motor (Duck) CW
     */

    /*
     * ********
     * gamepad2
     * ********
     * left_stick_y = arm up/down
     * dpad_up = arm to pos 3
     * dpad_left = arm to pos 2
     * dpad_down = arm to pos 1
     * X = open/close Lid
     * B = open Lid (but even more)
     * Y = stop arm movement (stop seizure)
     * right_trigger = flip (for both drivers)
     */


    private DcMotor WHEEL_FR = null;
    private DcMotor WHEEL_BR = null;
    private DcMotor WHEEL_FL = null;
    private DcMotor WHEEL_BL = null;
    private DcMotor TTS_Motor = null;
    private DcMotor arm = null;
    private DcMotor Flapper = null;
    private DigitalChannel Linebreak;

    private ElapsedTime runtime = new ElapsedTime();

    private DistanceSensor TOF_Front = null;
    private DistanceSensor TOF_Left = null;
    private DistanceSensor TOF_Right = null;


    private DcMotor IntakeArm = null;
    private Servo Lid = null; // PreLoaded Block Dropper
    private Servo Eyes = null;

    final double NormalDriveSpeed = 0.7; // Max Throttle for the Drive Motors Normally - Value Between 0 and 1
    final double NormalCrabSpeed = 0.8; // Max Throttle for the Drive Motors Normally - Value Between 0 and 1
    final double NormalTurnSpeed = 0.6; // Max Throttle for the Drive Motors Normally - Value Between 0 and 1
    final double MicroDriveSpeed = 0.1725; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MicroCrabSpeed = 0.45; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MicroTurnSpeed = 0.3; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MidDriveSpeed = 0.35; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MidCrabSpeed = 0.4; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MidTurnSpeed = 0.4; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double TTSpeed = 1; // Speed for the Turn Table Drive Motors - Value Between 0 and 1
    final double FlapperSpeed = 0.85;
    
    final double BoxOpenPos = 0.8;
    final double BoxFullOpenPos = 0.9;
    final double BoxClosedPos = 0.2;
    final double BoxOpenTopLevel = 0.8;
    final double BoxOpenMiddleLevel = 0.8;
    final double BoxOpenBottomLevel = 0.8;
    final double BoxOpenCapDown = 0.3;
    final double BoxOpenCapUp = 0.2;
    
    
    double IntakeElapsed;
    
    
    double DriveSpeed;
    double CrabSpeed;
    double TurnSpeed;
    
    
    String Bob = "HAPPY";
    
    int IntakeDownPos = -950;
    int IntakeUpPos = 0;
    double IntakeSpeed = 1;
    
    double Frontcm;
    double Leftcm;
    double Rightcm;

    double gamepad2Pressed;


    final double ArmSpeed = 0.5; //1
    final double MicroArmSpeed = 0.2; //0.4
    boolean armReachedTarget = true;
    double ArmTicksPerRot;
    // [0]: ticks for ground level, [1]: ticks for level 1, [2]: ticks for level 2, [3]: ticks for level 3
    // [4]: ticks for level 3 (flipped), [5]: ticks for level 2 (flipped), [6]: ticks for level 1 (flipped), [7]: ticks for ground level (flipped)
    int[] ArmLevelTicks;

    boolean armOpen = false;
    boolean intakeUp = true;
    boolean intakeSet = false;
    boolean boxUp = false;
    boolean IntakeArmGoUp = false;
    boolean IntakeSetUp = false;
    boolean ArmDown = true;
    boolean GotBlock = false;
    boolean BlockInBox = true;
    boolean PickUpBlock = false;
    boolean intakeCheck = false;
    
    
    
    boolean xButtonDown1;
    boolean yButtonDown1;
    boolean aButtonDown1;
    boolean bButtonDown1;
    boolean xButtonDown2;
    boolean yButtonDown2;
    boolean aButtonDown2;
    boolean bButtonDown2;
    boolean rightBumperDown;
    

  //public ElapsedTime mRunTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        
        

        WHEEL_FR = hardwareMap.get(DcMotor.class, "wheelfr"); // Front Right
        WHEEL_BR = hardwareMap.get(DcMotor.class, "wheelbr"); // Back Right
        WHEEL_FL = hardwareMap.get(DcMotor.class, "wheelfl"); // Front Left
        WHEEL_BL = hardwareMap.get(DcMotor.class, "wheelbl"); // Back Left
        TTS_Motor = hardwareMap.get(DcMotor.class, "duck");   // Motor for Duck Turn Table Spinner
        Flapper = hardwareMap.get(DcMotor.class, "Flapper");
        IntakeArm = hardwareMap.get(DcMotor.class, "IntakeArm"); //Arm with flapper
        Lid = hardwareMap.get(Servo.class, "Lid");         // Severo to drop pre loaded block drop
        Eyes = hardwareMap.get(Servo.class, "Eyes");
        TOF_Left = hardwareMap.get(DistanceSensor.class, "TOF_Left");   // Time of Flight Sensor mounted on the left of the robot
        TOF_Right = hardwareMap.get(DistanceSensor.class, "TOF_Right");   // Time of Flight Sensor mounted on the right of the robot
        TOF_Front = hardwareMap.get(DistanceSensor.class, "TOF_Front");// Time of Flight Sensor mounted on the front of the robot
        Linebreak = hardwareMap.get(DigitalChannel.class, "Linebreak");

        TTS_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        TTS_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        Flapper.setDirection(DcMotorSimple.Direction.FORWARD);

        arm = hardwareMap.dcMotor.get("Arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        WHEEL_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeArm.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        setArmMotorType(ArmMotorType.Rev20);



         Frontcm = TOF_Front.getDistance(DistanceUnit.CM);
         if(Frontcm < 15){
             MoveIntake(950, 0.6);
             IntakeDownPos = 0;
             IntakeUpPos = 950;
             telemetry.addData("WARNING", "Robot did not start within 18 inches!");
         }else{
             IntakeDownPos = -950;
             IntakeUpPos = 0;
         }
        boxUp = false;
        ArmDown = true;
         
        telemetry.update();

        //mRunTime.reset();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Set the default motor directions, Left and Right Wheel Motors are Flipped in Their Mounting, so Their Default Dirrections are Opposites
            WHEEL_FL.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
            WHEEL_BL.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);
            
            

            if (gamepad1.right_bumper) {
                DriveSpeed = MicroDriveSpeed;
                CrabSpeed = MicroCrabSpeed;
                TurnSpeed = MicroTurnSpeed;
            }
            else if (gamepad1.left_bumper) {
                DriveSpeed = NormalDriveSpeed;
                CrabSpeed = NormalCrabSpeed;
                TurnSpeed = NormalTurnSpeed;
            }
            else {
                
                DriveSpeed = MidDriveSpeed;
                CrabSpeed = MidCrabSpeed;
                TurnSpeed = MidTurnSpeed;
            }

            
            BlockInBox = Linebreak.getState();
            GotBlock = !BlockInBox;
            
          

            //Run the Wheels bases on the Joystick Positions on GamePad1

                //Drive version 1 (Left Joystick controls forward/backward and rotation, Right Joystick controls crabbing)
                /*
                WHEEL_FR.setPower(+(DriveSpeed * gamepad1.left_stick_y + DriveSpeed * gamepad1.left_stick_x + CrabSpeed * gamepad1.right_stick_x));
                WHEEL_BR.setPower(+(DriveSpeed * gamepad1.left_stick_y + DriveSpeed * gamepad1.left_stick_x - CrabSpeed * gamepad1.right_stick_x));
                WHEEL_FL.setPower(+(DriveSpeed * gamepad1.left_stick_y - DriveSpeed * gamepad1.left_stick_x - CrabSpeed * gamepad1.right_stick_x));
                WHEEL_BL.setPower(+(DriveSpeed * gamepad1.left_stick_y - DriveSpeed * gamepad1.left_stick_x + CrabSpeed * gamepad1.right_stick_x));
                */

                //Drive version 2 (Left Joystick controls forward/backward and crabbing, Right Joystick controls rotation)
                ///*
                
                
                
                
                
                
                //if(gamepad2.b){
                
                WHEEL_FR.setPower(+(DriveSpeed * gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BR.setPower(+(DriveSpeed * gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_FL.setPower(+(DriveSpeed * gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BL.setPower(+(DriveSpeed * gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                //*/
                
                
                
                
                //Drive version 3 (Left Joystick (flipped) controls forward/backward and crabbing, Right Joystick controls rotation)
                /*
                WHEEL_FR.setPower(+(DriveSpeed * -gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BR.setPower(+(DriveSpeed * -gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_FL.setPower(+(DriveSpeed * -gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BL.setPower(+(DriveSpeed * -gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                */
                
                
                
                
                

            // Not sure what this does
//            telemetry.addData("left y ", gamepad1.left_stick_y);
//            telemetry.addData("left x ", gamepad1.left_stick_x);
//            telemetry.addData("right x ", gamepad1.right_stick_x);
//            telemetry.update();


            
            
            
            //}



            //Bobs personality
            
//#################################################################################//            
            
            //ADVANCED AI (DO NOT TOUCH)
    
                    
    
            if(Bob == "HAPPY"){
                Eyes.setPosition(0.2);
                telemetry.addData("Bob", "Happy");
            }else if(Bob == "SAD"){
                Eyes.setPosition(0.5);
                telemetry.addData("Bob", "Sad");
            }else if(Bob == "WORRIED"){
                Eyes.setPosition(0.3);
                telemetry.addData("Bob", "Worried");
            }else if(Bob == "MAD"){
                Eyes.setPosition(0.1);
                telemetry.addData("Bob", "Mad");
            }else if(Bob == "ANGRY"){
                Eyes.setPosition(0.05);
                telemetry.addData("Bob", "Angry");
            }
            
            
            
            telemetry.addData("Line Break Sensor", Linebreak.getState());

            
            //Add telemetry here
            telemetry.addData("Lid Open", armOpen);
            telemetry.addData("Have Block", GotBlock);
            
            //telemetry.addData("Gamepad-trigger", gamepad1.right_trigger);
            
            
            
            telemetry.update();


//#################################################################################//         




            if(arm.getCurrentPosition()<20){
                ArmDown = true;
            }else{
                ArmDown = false;
            }



            // Run the Turn Table Spinner (Duck Motor) if the left or right triggers on GamePad1 are pressed
            if (gamepad1.right_trigger > 0.1) {
                TTS_Motor.setPower(+TTSpeed * gamepad1.right_trigger + 0.2);
                Bob = "MAD";
            } else if (gamepad1.left_trigger > 0.1) {
                TTS_Motor.setPower(-TTSpeed * gamepad1.left_trigger + 0.2);
                Bob = "MAD";
            } else if(gamepad1.right_stick_x != 0) {
                TTS_Motor.setPower(-TurnSpeed * gamepad1.right_stick_x * 0.4);
            }else if(GotBlock){
                TTS_Motor.setPower(0.3);
            }else{
                TTS_Motor.setPower(0);
            }


/*
            if(gamepad1.dpad_up){
                Bob = "HAPPY";
            }else if(gamepad1.dpad_left){
                Bob = "WORRIED";
            }else if(gamepad1.dpad_right){
                Bob = "SAD";
            }else if(gamepad1.dpad_down){
                Bob = "ANGRY";
            }
*/



            // ---
            // Second Controller
            
            
            // Run the flapper intake if the left or right triggers on GamePad1 are pressed
            
            if (gamepad2.right_trigger > 0.1) {
                Flapper.setPower(-FlapperSpeed);
                Bob = "ANGRY";
            } else if (gamepad2.left_trigger > 0.1) {
                Flapper.setPower(+FlapperSpeed);
                Bob = "MAD";
            } else
            //Runs the flapper when lid is opening and closing near it
            if(ArmDown && gamepad2.x){
                if(armOpen){
                if(!intakeUp){
                Flapper.setPower(0.6);
                }
                }else{
                if(!intakeUp){
                Flapper.setPower(-0.6);
                }
                }
            }else if(IntakeArmGoUp && arm.getCurrentPosition() < ArmLevelTicks[4] && arm.getCurrentPosition() > 0){
                Flapper.setPower(-0.6);
            }else if(PickUpBlock){
                
            }else {
                Flapper.setPower(0);
            }
            
            
            
            
            if(ArmDown){
                
            }
            
            // Run the pre loaded block dropper servo using GamePad1's X & B buttons
            if (gamepad2.x && !xButtonDown2) {
                armOpen = !armOpen;
                if (armOpen) {
                    if(arm.getCurrentPosition() > ArmLevelTicks[3] - 10 && arm.getCurrentPosition() < ArmLevelTicks[3]+10){ //Top
                        Lid.setPosition(BoxOpenTopLevel);
                        Bob = "WORRIED";
                    }else if(arm.getCurrentPosition() > ArmLevelTicks[2] - 10 && arm.getCurrentPosition() < ArmLevelTicks[2] + 10){ //Middle
                        Lid.setPosition(BoxOpenMiddleLevel);
                        Bob = "SAD";
                    }else if(arm.getCurrentPosition() > ArmLevelTicks[1] - 10){ //Bottom
                        Lid.setPosition(BoxOpenBottomLevel);
                        Bob = "HAPPY";
                    }else{
                        
                        Lid.setPosition(BoxFullOpenPos);
                        Bob = "HAPPY";
                        
                    }
                }
                else {
                    Lid.setPosition(BoxClosedPos);
                    Bob = "MAD";
                }
            }
            

            
            
            
            
            if(ArmDown && armOpen && intakeUp){
                if(gamepad1.a || gamepad2.a){
                Lid.setPosition(BoxOpenPos);
                armOpen = true;
                }
            }
            
            
            
            
            if(gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right){
                
            }
            
            
            if(ArmDown && !intakeUp){
                if(gamepad1.y || gamepad2.y){
                    if(boxUp){
                        Flapper.setPower(0.2);
                    }else{
                        Flapper.setPower(-0.2);
                    }
                }
            }
            
            
            if (arm.getCurrentPosition() < 300){
                if(gamepad1.y && !yButtonDown1){
                boxUp = !boxUp;
                if (boxUp) {
                    moveArm(180, 0.25);
                    Bob = "HAPPY";
                }
                else {
                    moveArm(0, 0.25);
                    Bob = "ANGRY";
                }
                }else
                if(gamepad2.y && !yButtonDown2){
                boxUp = !boxUp;
                if (boxUp) {
                    moveArm(180, 0.25);
                    Bob = "HAPPY";
                }
                else {
                    moveArm(0, 0.25);
                    Bob = "ANGRY";
                }
                }
            }
            
            
            
            
            
            
            
            
            
            
            if(!boxUp){
            if (gamepad1.a && !aButtonDown1) {
                intakeUp = !intakeUp;
                if (intakeUp) {
                    MoveIntake(IntakeUpPos, IntakeSpeed);
                    Bob = "HAPPY";
                }
                else {
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                    Bob = "MAD";
                }
            }else
            if (gamepad2.a && !aButtonDown2) {
                intakeUp = !intakeUp;
                if (intakeUp) {
                    MoveIntake(IntakeUpPos, IntakeSpeed);
                    Bob = "HAPPY";
                }
                else {
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                    Bob = "MAD";
                }
            }
            }
            
            
            
            
            
            
            
            
            
            aButtonDown1 = gamepad1.a;
            bButtonDown1 = gamepad1.b;
            yButtonDown1 = gamepad1.y;
            xButtonDown1 = gamepad1.x;
          
            aButtonDown2 = gamepad2.a;
            bButtonDown2 = gamepad2.b;
            yButtonDown2 = gamepad2.y;
            xButtonDown2 = gamepad2.x;
            
     

            // arm flipping


            rightBumperDown = gamepad2.right_bumper;

/*
            if (gamepad2.left_bumper) {
                telemetry.addData("arm encoder val", arm.getCurrentPosition());
                telemetry.addData("timestamp", gamepad2.timestamp);
                telemetry.update();
            }
*/
        
            
            if(IntakeArmGoUp && arm.getCurrentPosition() > ArmLevelTicks[4]+100){
                intakeUp = true;
                MoveIntake(IntakeUpPos, IntakeSpeed);
                //IntakeArm.setPosition(0);
                IntakeArmGoUp = false;
            }
            
            if(IntakeArmGoUp && arm.getCurrentPosition()<100){
                Flapper.setPower(0.5);
            }
            
            if(IntakeSetUp && arm.getCurrentPosition() < 20){
                IntakeSetUp = false;
            }
            
            if(gamepad2.left_bumper){
                if(PickUpBlock){
                    PickUpBlock = false;
                }
            }
            
            
            //picking up block
            if(gamepad2.right_bumper){
                if(!PickUpBlock){
                gamepad2Pressed = gamepad2.timestamp;
                }
                if(!GotBlock && arm.getCurrentPosition() <= ArmLevelTicks[7] + 10 && !PickUpBlock){ 
                PickUpBlock = true;
                intakeCheck = true;
                runtime.reset();
                }else if(!ArmDown && arm.getCurrentPosition() > ArmLevelTicks[7]){
                    MoveIntake(IntakeUpPos, IntakeSpeed);
                    intakeUp = true;
                    moveArm(ArmLevelTicks[7], ArmSpeed); 
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                }
                /*
                if(PickUpBlock && !GotBlock && gamepad2.timestamp > gamepad2Pressed + 50){
                PickUpBlock = false;
                }
                */
            }
            if(PickUpBlock){
                if(ArmDown && !boxUp){
                if(!intakeUp && IntakeArm.getCurrentPosition() > IntakeDownPos - 20){
                if(!GotBlock){
                Flapper.setPower(-FlapperSpeed);
                runtime.seconds();
                }else if(GotBlock){
                    if(runtime.seconds() < 2 && intakeCheck){
                        Flapper.setPower(FlapperSpeed * 0.6);
                    }else{
                    intakeCheck = false;
                    }
                    if(!intakeCheck){
                    IntakeArmGoUp = true;
                    moveArm(ArmLevelTicks[7], ArmSpeed);
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    PickUpBlock = false;
                    runtime.reset();
                    
                    
                    }
                }
                }else{
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                    Lid.setPosition(BoxOpenPos);
                    armOpen = true;
                }
                }else{
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                    intakeUp = false;
                    Lid.setPosition(BoxOpenPos);
                    armOpen = true;
                    if(arm.getCurrentPosition() > ArmLevelTicks[7]){
                        moveArm(0, ArmSpeed);
                        //PickUpBlock = false;
                    }else
                    if(IntakeArm.getCurrentPosition() > IntakeDownPos * 0.5){
                        moveArm(0, ArmSpeed);
                        //PickUpBlock = false;
                    }
                
                }
            }
            
            
            
            
            
           if(gamepad2.b){ 
            
            //capping
            if(gamepad1.dpad_up){
                if(arm.getCurrentPosition() > ArmLevelTicks[4]){
                    moveArm(ArmLevelTicks[6], MicroArmSpeed);
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(!intakeUp && IntakeArm.getCurrentPosition() < IntakeDownPos + 50){
                    moveArm(ArmLevelTicks[6], ArmSpeed);
                    IntakeArmGoUp = true;
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(intakeUp){
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                }
            }else if(gamepad1.dpad_down){
                if(arm.getCurrentPosition() > ArmLevelTicks[4]){
                    moveArm(ArmLevelTicks[5], MicroArmSpeed);
                    Bob = "SAD";
                    Lid.setPosition(BoxOpenCapDown);
                    armOpen = true;
                }else if(!intakeUp && IntakeArm.getCurrentPosition() < IntakeDownPos + 50){
                    moveArm(ArmLevelTicks[5], ArmSpeed);
                    IntakeArmGoUp = true;
                    Lid.setPosition(BoxOpenCapDown);
                    armOpen = true;
                    Bob = "SAD";
                }else if(intakeUp){
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                }
            }
        
            
            
        }
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            

            // arm presets
            if (gamepad2.dpad_up) {
                if(arm.getCurrentPosition() > ArmLevelTicks[4]){
                    moveArm(ArmLevelTicks[3], MicroArmSpeed);
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(!intakeUp && IntakeArm.getCurrentPosition() < IntakeDownPos + 50){
                    moveArm(ArmLevelTicks[3], ArmSpeed);
                    IntakeArmGoUp = true;
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(intakeUp){
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                }
            }
            if (gamepad2.dpad_left) {
                if(arm.getCurrentPosition() > ArmLevelTicks[4]){
                    moveArm(ArmLevelTicks[2], MicroArmSpeed);
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(intakeUp == false && IntakeArm.getCurrentPosition() < IntakeDownPos + 50){
                    moveArm(ArmLevelTicks[2], ArmSpeed);
                    IntakeArmGoUp = true;
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(intakeUp){
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                }
            }
            if (gamepad2.dpad_down) {
                if(arm.getCurrentPosition() > ArmLevelTicks[4]){
                    moveArm(ArmLevelTicks[1], MicroArmSpeed);
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(intakeUp == false && IntakeArm.getCurrentPosition() < IntakeDownPos + 50){
                    moveArm(ArmLevelTicks[1], ArmSpeed);
                    IntakeArmGoUp = true;
                    Lid.setPosition(BoxClosedPos);
                    armOpen = false;
                    Bob = "SAD";
                }else if(intakeUp){
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                }
                
            }
            if (gamepad2.dpad_right) {
                if(arm.getCurrentPosition() <= ArmLevelTicks[3]+10){
                moveArm(ArmLevelTicks[0], ArmSpeed * 0.5);
                }else{
                moveArm(ArmLevelTicks[0], ArmSpeed); 
                }
                IntakeSetUp = true;
                boxUp = false;
                intakeUp = false;
                MoveIntake(IntakeDownPos, IntakeSpeed);
                Lid.setPosition(BoxOpenPos);
                armOpen = true;
                armOpen = true;
                Bob = "ANGRY";
            }
            
            
            /*
            if (gamepad2.b) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armReachedTarget = true;
            }
            */
/*
            if (armReachedTarget) {
                arm.setPower(-gamepad2.left_stick_y * ArmSpeed);
                IntakeArm.setPower(gamepad2.right_stick_y * 0.5);
                if(arm.getCurrentPosition() <  ArmLevelTicks[4] * 3 && gamepad2.left_stick_y > 0){
                    intakeUp = false;
                    MoveIntake(IntakeDownPos, IntakeSpeed);
                }
            }
            */
            
            // check with 2 error margin
            else if (arm.getCurrentPosition() > arm.getTargetPosition() - 2 && arm.getCurrentPosition() < arm.getTargetPosition() - 2)
                armReachedTarget = true;
        }
    }

    enum ArmMotorType {
        Andy20,
        Rev20,
        Rev40,
        Rev60,
        Tetrix,
    }

    private void setArmMotorType(ArmMotorType type) {
        switch (type) {
            case Andy20:
                ArmTicksPerRot = 537.6;
                ArmLevelTicks = new int[] {100, 250, 500, 750};
                break;
            case Rev20:
                ArmTicksPerRot = 560;
                ArmLevelTicks = new int[]{0, 1100, 900, 800, 180, 1160, 750, 550};
                break;
            case Rev60: // same settings as 'REV20'
                ArmTicksPerRot = 1680.0;
                ArmLevelTicks = new int[] {100, 250, 500, 750};
                break;
            case Rev40:
                ArmTicksPerRot = 1120.0;
                ArmLevelTicks = new int[] {100, 250, 500, 750};
                break;
            case Tetrix:
                ArmTicksPerRot = 1440;
                //ArmLevelTicks = new int[]{-10, -300, -550, -850};
                ArmLevelTicks = new int[]{0, 2800, 2550, 2200, 2000};
                break;
        }
    }

    private void moveArm(int ticks, double speed) {
        arm.setTargetPosition(ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        arm.setPower(speed);
        
        
        armReachedTarget = false;
    }
    
    
    private void MoveIntake(int ticks, double speed) {
        IntakeArm.setTargetPosition(ticks);
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        IntakeArm.setPower(speed);
        
        
        
    }
}
