/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package AutonomousCodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TFEncoderLeftBlue", group = "Concept")

public class TFEncoderLeftBlue extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;
    Servo clamp;
    Servo armswivel;
   // DcMotor container;
DcMotorEx launcher;
 DcMotorEx launcher2;
Servo containerarm;
 DcMotor intake;
//Servo aimer;
    private BNO055IMU imu = null;
    private ElapsedTime     runtime = new ElapsedTime();

static final double     COUNTS_PER_MOTOR_REV    = 383.6 * 2;    // eg: TETRIX Motor Encoder
static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    
    
    //bottom lines of code are for encoder ticks in inches for cascading slides for stone lift in auton

    
    
    static final double     SLOW_DRIVE_SPEED        = 0.4;// is the speed when going straight in encoder drive
    static final double     FAST_DRIVE_SPEED        = 0.8;
    static final double     MED_DRIVE_SPEED         = 0.5;
    static final double     STRAFE_SPEED            = 0.7;
    static final double     FULL_THROTTLE        = 1;
    
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUUmFe7/////AAABmfrACDmLpkwPoZBcpjPAgD81Dd54BbEwE+bWlxUPFMMkELMLgNDtnLZm+XzK22G9gFzhzAFWBxFKtcRD3QkU/SrRc6nIQOMLVN+TgY7oL0V7/a5zdjQs7YFaxwGbrrYtvfThBqLBVq7fHOYdiLb24HaGxQuX6mwbRYZfV9vVRH8WtHhsPTi3J/tX4IG+oiyT8H22KqLfIy5ab0R1FJ2CyrvVYsJ7ogjKCz7lkf6BUMkVn7D/8QXUSoVfkoV4Vf5E966V7YfuXsLjQ1AhvcQuDnrXzh0EE9C/U+Ckux4WYTx7kTpmu5z80gQVqwdLQC6UTXVhvz3+ImkWr08GxXk+5m20Futx7YgxbyFYrNJXiBb2\n";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException{
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
         lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        armswivel = hardwareMap.servo.get("armswivel");
        clamp = hardwareMap.servo.get("clamp");
        intake = hardwareMap.dcMotor.get("intake");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        clamp.setPosition(.9);
        armswivel.setPosition(.18);
        //armswivel.setPosition(.74);
        //container = hardwareMap.dcMotor.get("container");
     //container.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //container.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //container.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //container.setTargetPosition(0);
    //container.setDirection(DcMotorSimple.Direction.REVERSE);
    launcher = hardwareMap.get(DcMotorEx.class, "launcher"); 
 launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 launcher.setDirection(DcMotorSimple.Direction.REVERSE);
 launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
 launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 
 
 //aimer = hardwareMap.servo.get("aimer");
 containerarm = hardwareMap.servo.get("containerarm");
 containerarm.setPosition(0.17);
   launcher.setVelocityPIDFCoefficients(1.22, .122, 0.0, 12.2);
   launcher.setPositionPIDFCoefficients(5.0);
   launcher2.setPositionPIDFCoefficients(5.0);
  
    launcher2.setVelocityPIDFCoefficients(1.22, .122, 0.0, 12.2);
    launcher.setPositionPIDFCoefficients(5.0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
http://192.168.43.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/TeamDrive.java        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imumain");
        imu.initialize(parameters);
         initVuforia();
        initTfod(); 
       tfod.activate();


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
        

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(1.78, 16/9);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 0 ){
                          // empty list.  no objects recognized.
                          telemetry.addData("TFOD", "No items detected.");
                          telemetry.addData("Target Zone", "A");
                          tfod.shutdown();
                       
                          
                          encoderDrive(FAST_DRIVE_SPEED, 39, 39, 39, 39, 10.0);
                          drive(0, 0, 0, 0, 200);
                          encoderDrive(MED_DRIVE_SPEED, -4, 4, 4, -4, 4.0);
                          drive(0, 0, 0, 0, 600);
                           //encoderDrive(FAST_DRIVE_SPEED, -12, 12, -12, 12, 5.0); //first turn
                           //encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 3.0);//go forward
                           armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 500);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 300);
                           launcher.setVelocity(2125);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
                            launcher2.setVelocity(2125);
                            sleep(500);
                           encoderDrive(FAST_DRIVE_SPEED, 7, -7, -7, 7, 5.0);
                           encoderDrive(FAST_DRIVE_SPEED, -7, -7, -7, -7, 4.0);
containerarm.setPosition(.34);//first ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(300);
//encoderDrive(FAST_DRIVE_SPEED, 5, -5, -5, 5, 3.0);
containerarm.setPosition(.34);//second ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(300);
//encoderDrive(FAST_DRIVE_SPEED, 5, -5, -5, 5, 3.0);
//encoderDrive(FAST_DRIVE_SPEED, -2, -2, -2, -2, 3.0);
drive(0, 0, 0, 0, 100);
containerarm.setPosition(.34);//third ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);
launcher.setVelocity(0);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
launcher2.setVelocity(0);
sleep(500);
encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 5.0);
//encoderDrive(SLOW_DRIVE_SPEED, 15, -15, -15, 15, 5.0);
clamp.setPosition(.9);
sleep(400);
        armswivel.setPosition(.18);
        sleep(400);
/*encoderDrive(FAST_DRIVE_SPEED, -22, -22, -22, -22, 5.0);
armswivel.setPosition(.72);
sleep(200);
encoderDrive(MED_DRIVE_SPEED, -5.2, 5.2, 5.2, -5.2, 4.0);
drive(0, 0, 0, 0, 900);
clamp.setPosition(.9);
sleep(500);
armswivel.setPosition(.5);
sleep(400);
encoderDrive(FAST_DRIVE_SPEED, 31, 31, 31, 31, 6.0);
encoderDrive(FAST_DRIVE_SPEED, -18, 18, 18, -18, 6.0);
armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 300);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 300);
encoderDrive(FAST_DRIVE_SPEED, 4, -4, -4, 4, 3.0);
clamp.setPosition(.9);
sleep(500);
armswivel.setPosition(.5);
sleep(400);
/*containerarm.setPosition(.34);//first ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(800);
launcher.setVelocity(0);
launcher2.setVelocity(0);
sleep(600);
encoderDrive(FAST_DRIVE_SPEED, 5, 5, 5, 5, 5.0);
                           /*encoderDrive(FAST_DRIVE_SPEED, 25, -25, -25, 25, 14.0);
                           drive(0, 0, 0, 0, 200);
                           encoderDrive(FAST_DRIVE_SPEED, -31, -31, -31, -31, 14.0);
                           drive(0, 0, 0, 0, 300);
                           encoderDrive(SLOW_DRIVE_SPEED, -2, 2, 2, -2, 3.0);
                           clamp.setPosition(.9);
                            drive(0, 0, 0, 0, 700);
                            armswivel.setPosition(.5);
                          /* encoderDrive(MED_DRIVE_SPEED, 8, -8, -8, 8, 5.0);
                           encoderDrive(MED_DRIVE_SPEED, -8, -8, -8, -8, 5.0);
                           encoderDrive(MED_DRIVE_SPEED, 2, -2, 2, -2, 4.0);
                           launcher.setVelocity(3000);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
                           sleep(1000);
containerarm.setPosition(.2);//first ring shot out
sleep(400);
containerarm.setPosition(0);
sleep(400);
containerarm.setPosition(.2);//second ring shot out
sleep(400);
containerarm.setPosition(0);
sleep(400);
containerarm.setPosition(.2);//third ring shot out
sleep(400);
containerarm.setPosition(0);
sleep(800);
launcher.setVelocity(0);
                           //encoderDrive(MED_DRIVE_SPEED, -10, -10, -10, -10, 5.0);
                           //drive(0, 0, 0, 0, 200);
                          /* launcher.setVelocity(3000);
                            sleep(1000);
                            encoderDrive(FAST_DRIVE_SPEED, 12, -12, 12, -12, 5.0);
                            drive(0, 0, 0, 0, 200);
                            encoderDrive(SLOW_DRIVE_SPEED, -2, -2, -2, -2, 3.0);
                            drive(0, 0, 0, 0, 200);
                             aimer.setPosition(.6);
 container.setTargetPosition(3200);
 container.setPower(1);
 sleep(2000);
 while(opModeIsActive() && container.isBusy()){

containerarm.setPosition(.35);//first ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(400);
containerarm.setPosition(.35);//second ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(400);
containerarm.setPosition(.35);//third ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(800);
launcher.setVelocity(0);
container.setTargetPosition(0);
container.setPower(-.9);
drive(0, 0, 0, 0, 100);
encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 4.0);}*/
drive(0, 0, 0, 0, 400000);
 
                         
                           
                           
                          
                          } else {
                          // list is not empty.
                          // step through the list of recognitions and display boundary info.
                          int i = 0;
                          for (Recognition recognition : updatedRecognitions) {
                              telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                              telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                      recognition.getLeft(), recognition.getTop());
                              telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                      recognition.getRight(), recognition.getBottom());

                              // check label to see which target zone to go after.
                              if (recognition.getLabel().equals("Single")) {
                                  telemetry.addData("Target Zone", "B");
                                  tfod.shutdown();
                          /*armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 600);
                          encoderDrive(FAST_DRIVE_SPEED, -4, 4, 4, -4, 4.0);
                          drive(0, 0, 0, 0, 600);
                          encoderDrive(FULL_THROTTLE, 52, 52, 52, 52, 10.0);
                          drive(0, 0, 0, 0, 200);
                          armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 300);
                           clamp.setPosition(.5);
                           drive(0, 0, 0, 0, 300);
                           armswivel.setPosition(.5);
                           launcher.setVelocity(4000);
                           encoderDrive(FAST_DRIVE_SPEED, -19, -19, -19, -19, 6.0);
                           drive(0, 0, 0, 0, 100);
                           //driveHeadingTime(-.8, 0, 1.5);
                          encoderDrive(MED_DRIVE_SPEED, 21, -21, -21, 21, 5.0 );
                          aimer.setPosition(.63);
 container.setTargetPosition(3200);
 container.setPower(1);
 sleep(1300);
 while(opModeIsActive() && container.isBusy()){
//launcher.setVelocity(3000);
drive(0, 0, 0, 0, 300);
containerarm.setPosition(.35);//first ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(400);
encoderDrive(FAST_DRIVE_SPEED, 3, -3, -3, 3, 5.0);
containerarm.setPosition(.35);//second ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(400);
encoderDrive(FAST_DRIVE_SPEED, 5, -5, -5, 5, 5.0);
containerarm.setPosition(.35);//third ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(500);
launcher.setVelocity(0);
container.setTargetPosition(0);
container.setPower(-.9);
encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 4.0);
drive(0, 0, 0, 0, 400000);
 }*/
 //armswivel.setPosition(.5);
                          // drive(0, 0, 0, 0, 600);
                          // encoderDrive(MED_DRIVE_SPEED, 14, 14, 14, 14, 4.0);
                           //drive(0, 0, 0, 0, 100);
                         // encoderDrive(FAST_DRIVE_SPEED, 12, -12, -12, 12, 4.0);
                          //drive(0, 0, 0, 0, 300);
                          encoderDrive(FAST_DRIVE_SPEED, 1, 1, 1, 1, 3.0);
                          encoderDrive(MED_DRIVE_SPEED, -4, 4, 4, -4, 5.0);
                          encoderDrive(FAST_DRIVE_SPEED, 49, 49, 49, 49, 10.0);
                          encoderDrive(SLOW_DRIVE_SPEED, 3, 3, 3, 3, 4.0);
                          drive(0, 0, 0, 0, 200);
                          encoderDrive(STRAFE_SPEED, 16, -16, -16, 16, 4.0);
                          drive(0, 0, 0, 0, 200);
                         armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 700);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 500);
                           armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 100);
                           clamp.setPosition(.75);
                            drive(0, 0, 0, 0, 300);
                            launcher.setVelocity(2125); // sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
                            launcher2.setVelocity(2125);
                            sleep(600);
                            //'encoderDrive(MED_DRIVE_SPEED, -.2, .2, -.2, .2, 2.0);
                           encoderDrive(FAST_DRIVE_SPEED, -20, -20, -20, -20, 5.0);
                           drive(0, 0, 0, 0, 300);
                           encoderDrive(FAST_DRIVE_SPEED, -8, 8, 8, -8, 4.0);
                           drive(0, 0, 0, 0, 600);
                           intake.setPower(1);
sleep(400);
                      containerarm.setPosition(.34);//first ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(600);
//encoderDrive(FAST_DRIVE_SPEED, 5, -5, -5, 5, 3.0);
containerarm.setPosition(.34);//second ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(300);
//encoderDrive(FAST_DRIVE_SPEED, 5, -5, -5, 5, 3.0);
//encoderDrive(FAST_DRIVE_SPEED, -2, -2, -2, -2, 3.0);
drive(0, 0, 0, 0, 100);
containerarm.setPosition(.34);//third ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);
drive(0, 0, 0, 0, 300);
encoderDrive(FAST_DRIVE_SPEED, -12, -12, -12, -12, 5.0);
drive(0, 0, 0, 0, 600);
encoderDrive(FAST_DRIVE_SPEED, 12, 12, 12, 12, 5.0);
drive(0, 0, 0, 0, 600);
containerarm.setPosition(.34);//second ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);

launcher.setVelocity(0);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
launcher2.setVelocity(0);
intake.setPower(0);
sleep(500);
encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 5.0);
//encoderDrive(SLOW_DRIVE_SPEED, 15, -15, -15, 15, 5.0);
/*encoderDrive(FAST_DRIVE_SPEED, -22.4, -22.4, -22.4, -22.4, 5.0);
armswivel.setPosition(.72);
sleep(200);
clamp.setPosition(.4);
drive(0, 0, 0, 0, 500);
encoderDrive(MED_DRIVE_SPEED, -3.7, 3.7, 3.7, -3.7, 4.0);
drive(0, 0, 0, 0, 900);
clamp.setPosition(.9);
sleep(500);
armswivel.setPosition(.5);
sleep(400);
encoderDrive(FAST_DRIVE_SPEED, 42, 42, 42, 42, 8.0);
encoderDrive(FAST_DRIVE_SPEED, -6, 6, 6, -6, 4.0);
drive(0, 0, 0, 0, 200);
armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 300);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 500);
                           armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 100);
                           clamp.setPosition(.75);
                            drive(0, 0, 0, 0, 300);
encoderDrive(FAST_DRIVE_SPEED, 4, -4, -4, 4, 3.0);
encoderDrive(FAST_DRIVE_SPEED, -11, -11, -11, -11, 5.0);
                           /* encoderDrive(FAST_DRIVE_SPEED, -10, 10, 10, -10, 6.0);
                           drive(0, 0, 0, 0, 200);
                            launcher.setVelocity(1940);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
                            launcher2.setVelocity(1940);
                            intake.setPower(1);
                           sleep(500);
containerarm.setPosition(.34);//first ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);
containerarm.setPosition(.34);//second ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);
containerarm.setPosition(.34);//third ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);
containerarm.setPosition(.34);//first ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(800);
launcher.setVelocity(0);
launcher2.setVelocity(0);
intake.setPower(0);
sleep(600);
encoderDrive(FAST_DRIVE_SPEED, 11.5, -11.5, 11.5, -11.5, 4.0);
drive(0, 0, 0, 0, 300);
encoderDrive(FAST_DRIVE_SPEED, -6.8, -6.8, -6.8, -6.8, 5.0);
drive(0, 0, 0, 0, 200);
armswivel.setPosition(.67);
 drive(0, 0, 0, 0, 300);
clamp.setPosition(.4);
 drive(0, 0, 0, 0, 500);
encoderDrive(MED_DRIVE_SPEED, -14, 14, 14, -14, 7.0);
clamp.setPosition(.9);
drive(0, 0, 0, 0, 500);
 armswivel.setPosition(.5);
 drive(0, 0, 0, 0, 500);
 encoderDrive(FAST_DRIVE_SPEED, -3, -3, -3, -3, 5.0);
 encoderDrive(FAST_DRIVE_SPEED, 45, -45, -45, 45, 9.0);
 encoderDrive(MED_DRIVE_SPEED, -12, 12, -12, 12, 5.0);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 500);
encoderDrive(FAST_DRIVE_SPEED, -13, -13, -13, -13, 7.0);
                         
//encoderDrive(FAST_DRIVE_SPEED, 12, 12, 12, 12, 5.0);
                           
                          /*  encoderDrive(FAST_DRIVE_SPEED, 8, -8, -8, 8, 6.0);
                           // clamp.setPosition(.75);
                            drive(0, 0, 0, 0, 300);
                           armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 100);
                           encoderDrive(FAST_DRIVE_SPEED, -45, -45, -45, -45, 10.0); 
                           drive(0, 0, 0, 0, 100);
                            armswivel.setPosition(.67);
                            drive(0, 0, 0, 0, 100);
                            //clamp.setPosition(.4);
                            encoderDrive(SLOW_DRIVE_SPEED, -4, 4, 4, -4, 4.0);
                            drive(0, 0, 0, 0, 100);
                            clamp.setPosition(.9);
                            drive(0, 0, 0, 0, 700);
                            armswivel.setPosition(.5);
                            encoderDrive(FAST_DRIVE_SPEED, 47, 47, 47, 47, 10.0);
                            drive(0, 0, 0, 0, 200);
                            encoderDrive(MED_DRIVE_SPEED, -3, 3, 3, -3, 5.0);
                            armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 300);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 300);
                           armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 200);
                           encoderDrive(FAST_DRIVE_SPEED, -30, -30, -30, -30, 4.0);
                           drive(0, 0, 0, 0, 200);
                           encoderDrive(FAST_DRIVE_SPEED, -10, 10, 10, -10, 6.0);
                           drive(0, 0, 0, 0, 200);
                            //launcher.setVelocity(2650);
                           /*driveHeadingTime(-.8, 0, 1.5);
                          encoderDrive(MED_DRIVE_SPEED, 9, -9, -9, 9, 5.0 );
                       aimer.setPosition(.6);
 container.setTargetPosition(3200);
 container.setPower(1);
 sleep(1300);
 /*while(opModeIsActive() && container.isBusy()){
launcher.setVelocity(3000);
drive(0, 0, 0, 0, 300);
containerarm.setPosition(.35);//first ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(600);
//encoderDrive(FAST_DRIVE_SPEED, 3, -3, -3, 3, 5.0);
containerarm.setPosition(.35);//second ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(600);
//encoderDrive(FAST_DRIVE_SPEED, 5, -5, -5, 5, 5.0);
containerarm.setPosition(.35);//third ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(500);
launcher.setVelocity(0);
container.setTargetPosition(0);
container.setPower(-.9);
encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 4.0);}*/
drive(0, 0, 0, 0, 400000);
                          
                          
                                   
                              } else if (recognition.getLabel().equals("Quad")) {
                                  telemetry.addData("Target Zone", "C");
                                   tfod.shutdown();
                         
                          
                           encoderDrive(FAST_DRIVE_SPEED, 1, 1, 1, 1, 2.0);
                           drive(0, 0, 0, 0, 100);
                          encoderDrive(MED_DRIVE_SPEED, -4, 4, 4, -4, 4.0);
                          drive(0, 0, 0, 0, 600);
                           encoderDrive(FAST_DRIVE_SPEED, 60, 60, 60, 60, 10.0);
                          drive(0, 0, 0, 0, 200);
                         armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 700);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 500);
                           armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 300);
                           clamp.setPosition(.75);
                            drive(0, 0, 0, 0, 300);
                            launcher.setVelocity(2125);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
                            launcher2.setVelocity(2125);
                            
                           sleep(500);
                        encoderDrive(MED_DRIVE_SPEED, 9, -9, -9, 9, 5.0);
                           drive(0, 0, 0, 0, 200);
                           encoderDrive(FAST_DRIVE_SPEED, -28, -28, -28, -28, 7.0);
                          intake.setPower(1);
sleep(500);
                            
containerarm.setPosition(.34);//first ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);

containerarm.setPosition(.34);//second ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);

drive(0, 0, 0, 0, 100);

containerarm.setPosition(.34);//third ring shot out
sleep(200);
containerarm.setPosition(0.17);
sleep(500);
//encoderDrive(SLOW_DRIVE_SPEED, -12, -12, -12, -12, 6.0);
launcher.setVelocity(0);// sets the launching motor to a velocity of 3000 ticks/sec to reach max speed
launcher2.setVelocity(0);
intake.setPower(0);
sleep(500);
encoderDrive(SLOW_DRIVE_SPEED, 4, 4, 4, 4, 5.0);
//encoderDrive(SLOW_DRIVE_SPEED, 15, -15, -15, 15, 5.0);
/*encoderDrive(FAST_DRIVE_SPEED, -23, -23, -23, -23, 5.0);
armswivel.setPosition(.72);
sleep(200);
clamp.setPosition(.4);
drive(0, 0, 0, 0, 500);
encoderDrive(MED_DRIVE_SPEED, -3, 3, 3, -3, 4.0);
drive(0, 0, 0, 0, 900);
clamp.setPosition(.9);
sleep(500);
armswivel.setPosition(.5);
sleep(400);
encoderDrive(FULL_THROTTLE, 54, 54, 54, 54, 8.0);
drive(0, 0, 0, 0, 200);
encoderDrive(FULL_THROTTLE, -20, 20, 20, -20, 5.0);
drive(0, 0, 0, 0, 200);
                         armswivel.setPosition(.67);
                           drive(0, 0, 0, 0, 700);
                           clamp.setPosition(.4);
                           drive(0, 0, 0, 0, 500);
                           armswivel.setPosition(.5);
                           drive(0, 0, 0, 0, 200);
encoderDrive(FULL_THROTTLE, -23, -23, -23, -23, 5.0);

//encoderDrive(FAST_DRIVE_SPEED, 7, 7, 7, 7, 4.0);
                           /*encoderDrive(FAST_DRIVE_SPEED, -25, 25, 25, -25, 8.0);
                           encoderDrive(MED_DRIVE_SPEED, -11.5, -11.5, -11.5, -11.5, 5.0);
                           drive(0, 0, 0, 0, 200);
                           launcher.setVelocity(2650);
                            sleep(1000);
                            encoderDrive(FAST_DRIVE_SPEED, 12, -12, 12, -12, 5.0);
                            drive(0, 0, 0, 0, 200);
                            encoderDrive(SLOW_DRIVE_SPEED, -3, -3, -3, -3, 3.0);
                            drive(0, 0, 0, 0, 200);
                             //aimer.setPosition(.6);
 //container.setTargetPosition(3200);
 //container.setPower(1);
 //sleep(2000);
 /*while(opModeIsActive() && container.isBusy()){
containerarm.setPosition(.35);//first ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(300);
containerarm.setPosition(.35);//second ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(300);
containerarm.setPosition(.35);//third ring shot out
sleep(400);
containerarm.setPosition(.65);
sleep(800);
launcher.setVelocity(0);
container.setTargetPosition(0);
container.setPower(-.9);
drive(0, 0, 0, 0, 100);
encoderDrive(SLOW_DRIVE_SPEED, 5, 5, 5, 5, 4.0);}*/
drive(0, 0, 0, 0, 400000);
 
                         
                              } else {
                                  telemetry.addData("Target Zone", "UNKNOWN");
                              }
                          }
                      }

                      telemetry.update();
                    }
                }
            }
        
            
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "12398Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
public void encoderDrive(double speed, double leftfrontInches, double rightfrontInches, double leftrearInches, double rightrearInches, double timeoutS)// method to use encoders and either use drive or turn speed for distance and degree with encoders.
 {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = lf.getCurrentPosition() + (int)(leftfrontInches * COUNTS_PER_INCH);
            newRightFrontTarget =rf.getCurrentPosition() + (int)(rightfrontInches * COUNTS_PER_INCH);
            newLeftRearTarget =lr.getCurrentPosition() + (int)(leftrearInches * COUNTS_PER_INCH);
            newRightRearTarget =rr.getCurrentPosition() + (int)(rightrearInches * COUNTS_PER_INCH);
            lf.setTargetPosition(newLeftFrontTarget);
            lr.setTargetPosition(newLeftRearTarget);
            rf.setTargetPosition(newRightFrontTarget);
            rr.setTargetPosition(newRightRearTarget);
            
            

            // Turn On RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            

            // reset the timeout time and start motion.
            runtime.reset();
            lf.setPower(Math.abs(speed));
            rf.setPower(Math.abs(speed));
            lr.setPower(Math.abs(speed));
            rr.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

             while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (lf.isBusy() && rf.isBusy() && lr.isBusy() && rr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",lf.getCurrentPosition(), rf.getCurrentPosition(),lr.getCurrentPosition(), rr.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lf.setPower(0);
            rf.setPower(0);
            lr.setPower(0);
            rr.setPower(0);
            // Turn off RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
}
public void drive(double leftfrontspeed, double rightfrontspeed, double leftrearspeed, double rightrearspeed, int msec){
lf.setPower(leftfrontspeed);
rf.setPower(rightfrontspeed);
lr.setPower(leftrearspeed);
rr.setPower(rightrearspeed);
sleep(msec);
lf.setPower(0);
rf.setPower(0);
lr.setPower(0);
rr.setPower(0);
lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
}
public void driveHeadingTime(double throttle, double heading, double time) {
        runtime.reset();
        while (opModeIsActive()) {
            if (runtime.seconds() > time) break;
            driveHeading(throttle, heading);
        }
    }


    // This method adjusts the speed and direction of the robot to move towards a desired heading.
    // It's generally intended to be called repeatedly from the body of a 'while' or 'for' loop.
    //    (See example in driveHeadingTime(...) above.)
    public void driveHeading(double throttle, double heading) {
        // get the current angle of the robot
        double Z = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES).firstAngle;

        // display the robot's current angle (note: telemetry is slow)
         telemetry.addData("Big degree Z", Z);
         telemetry.update();

 // calculate how much turn is needed to approach heading
double turn = (Z - heading) * .05;

// change the power to the drive motors
lf.setPower(throttle + turn);
rf.setPower(throttle - turn);
lr.setPower(throttle + turn);
rr.setPower(throttle - turn);

lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
