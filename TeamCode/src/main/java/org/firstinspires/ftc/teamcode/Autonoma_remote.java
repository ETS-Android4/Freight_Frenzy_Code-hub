package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Autonoma_remote", group = "FTC")
public class Autonoma_remote extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Freight_frenzy.tflite";
    private static final String LABEL_FIRST_ELEMENT = "stanga-jos";
    private static final String LABEL_SECOND_ELEMENT = "mijloc-mijloc";
    private static final String LABEL_THIRD_ELEMENT = "dreapta-sus";

    /*
    0 stanga-jos
    1 mijloc-mijloc
    2 dreapta-sus
    */

    private static final String VUFORIA_KEY =
            "Aa9Z7Tv/////AAABmYjSlt0dS0duhcm6HAAAEXUlYhNDZlB6FiaOwGF7+5df23rR2+WRdBEskwZIKR4tQ3JnVR32xmN6zh21WA+P9nRBjjxRS25oC4bu4fwsX5h62FpVc0c+01jYm194wZl7GgIXYJ5NI9v77DlDpxK1ZOxA1UWmTL7JcxBZOa7wv7xDkCchdsy7bAUHlhf2Rrn+jTzrQXsl4ihm+VE4JUfOpccglsYZ0zrL9RGemllO4F1IvDqtcJoqI0+V7CBUMHMKsi1bOMmiwPK7O0Frtalte+m6aBB9NgjgjvrzZltyr9VZBtL3WGZuqBQ8MI/lYuYZyTreVAjk4iGFmJ2ycJwkQnO7ciXd7WgPlKmavx2NIH1f";

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
    private BNO055IMU imu;

    // movement motors
    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;

    //servos
    Servo carusel;
    Servo control;
    Servo retragere;

    DcMotor intake1;
    DcMotor intake2;
    DcMotor extindere;
    DcMotor glisiera;

    DistanceSensor distanta;

    private boolean didFunctionRun = false;

    private double width = 16.0; //inches
    private int cpr = 28; //counts per rotation
    private int gearratio = 28;
    private double diameter = 3.77;
    private double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    private double bias = 0.8;//default 0.8
    private double meccyBias = 0.9;//change to adjust only strafing movement
    //
    private double conversion = cpi * bias;
    private boolean exit = false;
    //
    private Orientation angles;
    private Acceleration gravity;

    @Override
    public void runOpMode() {
        initGyro();
//         The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//         first.
        //CALIBRARE GYRO START
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("IMU Calibration Status :", imu.getCalibrationStatus().toString());
        telemetry.update();

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }


        // sensor
        distanta = hardwareMap.get(DistanceSensor.class, "distanta");
        //servos
        retragere = hardwareMap.servo.get("retragere");
        carusel = hardwareMap.servo.get("carusel");
        control = hardwareMap.servo.get("control");

        //dc motors
        stangafata = hardwareMap.dcMotor.get("stangafata");
        dreaptafata = hardwareMap.dcMotor.get("dreaptafata");
        stangaspate = hardwareMap.dcMotor.get("stangaspate");
        dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        extindere = hardwareMap.dcMotor.get("extindere");
        glisiera = hardwareMap.dcMotor.get("glisiera");


        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);

        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reversePolarity();


        waitForStart();
        while (opModeIsActive()) {
            if (!didFunctionRun)
                doSmartStuff();
            else requestOpModeStop();
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void extindere(int level){
        if(level==1){
            //nivel jos
            glisiera.setTargetPosition(1850);
            glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            glisiera.setPower(1);

            extindere.setTargetPosition(1200);
            extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extindere.setPower(0.8);
            telemetry.addData("ajuns la nivel 3","");
        }

        if(level==2){
            //nivel mijloc
        }

        if(level==3){
            //nivel sus
        }
    }
    public void retragere(){
        extindere.setTargetPosition(10);
        extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extindere.setPower(0.8);

        glisiera.setTargetPosition(10);
        glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiera.setPower(1);
    }

    public void car_navigation(){
        if(distanta.getDistance(DistanceUnit.CM)<=36.5){
            stangafata.setPower(0);
            dreaptafata.setPower(0);
            stangaspate.setPower(0);
            dreaptaspate.setPower(0);
        }

    }
    public void carusel(int direction){
        //partea albastra
        carusel.setPosition(direction);
        //+ rosu
        //- albastru
        sleep(5500);
        carusel.setPosition(0.5);
    }

    private void caz_stanga() {

        moveToPosition(-15, 1);
        turnWithGyro(85,-0.75);
        moveToPosition(-60, 0.8);
        car_navigation();
        carusel(-1);

        strafeToPosition(-100,0.8);
        moveToPosition(65,0.85);
        turnWithGyro(180,-1);
        telemetry.addData("arrived at sh","placing preload");

        // extindere brat
        strafeToPosition(-70, 0.8);
        moveToPosition(-250,1);
        turnWithGyro(180,1);
        retragere.setPosition(0.7);
        telemetry.addData("Arrived in warehouse.", "lowered intake");

        requestOpModeStop();
        telemetry.update();

    }

    private void caz_mijloc() {
        telemetry.addData("caz","mijloc");

        moveToPosition(-15, 1);
        turnWithGyro(85,-0.75);
        moveToPosition(-60, 0.8);
        car_navigation();
        carusel(-1);

        strafeToPosition(-100,0.8);
        moveToPosition(60,1);
        turnWithGyro(180,-1);
        telemetry.addData("arrived at sh","placing preload");

        // extindere brat
        strafeToPosition(-70, 0.8);
        moveToPosition(-250,1);
        turnWithGyro(180,1);
        retragere.setPosition(0.7);
        telemetry.addData("Arrived in warehouse.", "lowered intake");

        requestOpModeStop();
        telemetry.update();
    }

    private void caz_dreapta() {
        telemetry.addData("caz","dreapta");

        moveToPosition(-15, 1);
        turnWithGyro(85,-0.75);
        moveToPosition(-60, 0.8);
        car_navigation();
        carusel(-1);

        strafeToPosition(-100,0.8);
        moveToPosition(60,1);
        turnWithGyro(180,-1);
        telemetry.addData("arrived at sh","placing preload");

        // extindere brat
        strafeToPosition(-70, 0.8);
        moveToPosition(-250,1);
        turnWithGyro(180,1);
        retragere.setPosition(0.7);
        telemetry.addData("Arrived in warehouse.", "lowered intake");

        requestOpModeStop();
        telemetry.update();
    }

    private boolean gotEm = false;
    private void doSmartStuff() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            sleep(750);
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && !gotEm) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                gotEm = true;
                if (updatedRecognitions.size() == 1) {
                    Recognition recognition = updatedRecognitions.get(0);
                    telemetry.addData("Recon", recognition.getLabel());
                    telemetry.update();
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        caz_mijloc();
                    } else
                        caz_dreapta();
                } else {
                    caz_stanga();
                }
                didFunctionRun = true;
            } else requestOpModeStop();
        }
    }
// cod nou
    //sall


    private void DoAutonomusStuff(boolean didFunctionRun){
        if(!didFunctionRun){
            //stanga-jos
            //dreapta-sus
            doSmartStuff();
            didFunctionRun = true;
        }
    }


    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the centimeter input negative.
     */
    public void moveToPosition(double cm, double speed){
        //
        double inches = cm / 2.54;
        int move = (int)(Math.round(inches*conversion));
        //
        stangaspate.setTargetPosition(stangaspate.getCurrentPosition() + move);
        stangafata.setTargetPosition(stangafata.getCurrentPosition() + move);
        dreaptaspate.setTargetPosition(dreaptaspate.getCurrentPosition() + move);
        dreaptafata.setTargetPosition(dreaptaspate.getCurrentPosition() + move);
        //
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        stangafata.setPower(speed);
        dreaptafata.setPower(speed);
        stangaspate.setPower(speed);
        dreaptaspate.setPower(speed);
        //
        while (stangafata.isBusy() && dreaptafata.isBusy() && stangaspate.isBusy() && dreaptaspate.isBusy()){
            if (exit){
                dreaptafata.setPower(0);
                stangafata.setPower(0);
                dreaptaspate.setPower(0);
                stangaspate.setPower(0);
                return;
            }
        }
        dreaptafata.setPower(0);
        stangafata.setPower(0);
        dreaptaspate.setPower(0);
        stangaspate.setPower(0);
        return;
    }

/*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        if (speedDirection > 0){//set target positions
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
            } else {
                first = devertify(yaw);
            }
            second = degrees + devertify(yaw);
        } else {
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
            } else {
                first = devertify(yaw);
            }
            second = devertify(-degrees + devertify(yaw));
        }
        //
        double firsta = convertify(first - 5);//175
        double firstb = convertify(first + 5);//-175
        turnWithEncoder(speedDirection);
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        double seconda = convertify(second - 5);//175
        double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            stangafata.setPower(0);
            dreaptafata.setPower(0);
            stangaspate.setPower(0);
            dreaptaspate.setPower(0);
        }
        //</editor-fold>
        //
        stangafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaspate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaspate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double cm, double speed){
        //
        double inches = cm / 2.54;
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //

        stangafata.setTargetPosition(stangafata.getCurrentPosition() + move);
        dreaptafata.setTargetPosition(dreaptafata.getCurrentPosition() - move);
        stangaspate.setTargetPosition(stangaspate.getCurrentPosition() - move);
        dreaptaspate.setTargetPosition(dreaptaspate.getCurrentPosition() + move);

        //
        stangafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptafata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //
        stangafata.setPower(speed);
        dreaptafata.setPower(speed);
        stangaspate.setPower(speed);
        dreaptaspate.setPower(speed);
        //
        while (stangafata.isBusy() && dreaptafata.isBusy() && stangaspate.isBusy() && dreaptaspate.isBusy()){}
        stangafata.setPower(0);
        dreaptafata.setPower(0);
        stangaspate.setPower(0);
        dreaptaspate.setPower(0);

        return;
    }

    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }


    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    public void turnWithEncoder(double input){
        stangafata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptafata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaspate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaspate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        stangafata.setPower(input);
        dreaptafata.setPower(-input);
        stangaspate.setPower(input);
        dreaptaspate.setPower(-input);
    }

    private void reversePolarity(){
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);


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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
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






}