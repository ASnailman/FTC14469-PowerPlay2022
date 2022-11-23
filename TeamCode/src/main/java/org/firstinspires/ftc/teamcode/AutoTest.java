package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "AutoTest", group = "MecanumDrive")
public class AutoTest extends LinearOpMode {

    //Control Hub Orientation
    byte AXIS_MAP_CONFIG_BYTE = 0x06; //rotates control hub 90 degrees around y axis by swapping x and z axis
    byte AXIS_MAP_SIGN_BYTE = 0x01; //Negates the remapped z-axis

    //Motors
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor RailRight;
    static DcMotor RailLeft;
    static DcMotor RotatingBase;
    static CRServo LeftClaw;
    static CRServo RightClaw;

    static NormalizedColorSensor rightColorsensor;
    static NormalizedColorSensor leftColorsensor;
    static NormalizedColorSensor centerRightColorsensor;
    static NormalizedColorSensor centerLeftColorsensor;
    //    static DistanceSensor frontRightDistanceSensor;
//    static DistanceSensor frontLeftDistanceSensor;
    static DistanceSensor backRightDistanceSensor;
    static DistanceSensor backLeftDistanceSensor;

    //Sensors
    BNO055IMU IMU;
    OpenCvWebcam webcam;
    VisionClassAutoRightBlue.SignalDeterminationPipeline pipeline;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    int programOrder = 0;
    double current_value;
    double prev_value = 0;
    double final_value;
    boolean turnright = false;
    boolean turnleft = false;
    boolean posOne;
    boolean posTwo;
    boolean posThree;

    boolean rightRed;
    boolean rightBlue;
    boolean rightUnknown;
    boolean leftRed;
    boolean leftBlue;
    boolean leftUnknown;
    boolean centerRightRed;
    boolean centerRightBlue;
    boolean centerRightUnknown;
    boolean centerLeftRed;
    boolean centerLeftBlue;
    boolean centerLeftUnknown;
    double distance;

    double frontRightDistance;
    double frontLeftDistance;
    double backRightDistance;
    double backLeftDistance;
    boolean leftFrontDistanceCleared = false;
    boolean rightFrontDistanceCleared = false;
    boolean leftBackDistanceCleared = false;
    boolean rightBackDistanceCleared = false;

    boolean frontLeftDistanceSamples_Reset = false;
    double[] frontLeftDistanceSamples = {819, 819, 819};
    boolean frontRightDistanceSamples_Reset = false;
    double[] frontRightDistanceSamples = {819, 819, 819};
    boolean backLeftDistanceSamples_Reset = false;
    double[] backLeftDistanceSamples = {122, 122, 122, 122, 122};
    boolean backRightDistanceSamples_Reset = false;
    double[] backRightDistanceSamples = {819, 819, 819};
    int frontLeftCount = 0;
    int frontRightCount = 0;
    int backLeftCount = 0;
    int backRightCount = 0;

    ElapsedTime ET = new ElapsedTime();

    public void runOpMode() {

        //Initialize the motors and sensors
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        LeftClaw = hardwareMap.get(CRServo.class, "leftClaw");
        RightClaw = hardwareMap.get(CRServo.class, "rightClaw");
        RailRight = hardwareMap.get(DcMotor.class, "RailRight");
        RailLeft = hardwareMap.get(DcMotor.class, "RailLeft");
        RotatingBase = hardwareMap.get(DcMotor.class, "RotatingBase");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        rightColorsensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
        leftColorsensor = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        centerRightColorsensor = hardwareMap.get(NormalizedColorSensor.class, "centerRightColorSensor");
        centerLeftColorsensor = hardwareMap.get(NormalizedColorSensor.class, "centerLeftColorSensor");
//        frontRightDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontRightDistanceSensor");
//        frontLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontLeftDistanceSensor");
        backRightDistanceSensor = hardwareMap.get(DistanceSensor.class, "backRightDistanceSensor");
        backLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backLeftDistanceSensor");

        //Configure the control hub orientation
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        //Rail Presets
        AttachmentMotorPresets();

        //Claw Presets
//        Claw.setDirection(Servo.Direction.FORWARD);
//        Claw.scaleRange(0, 1);
//        Claw.setPosition(1);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
//        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        //Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionClassAutoRightBlue.SignalDeterminationPipeline();
        webcam.setPipeline(pipeline);
        pipeline.InitTelemetry(telemetry);

//        webcam.setPipeline(new VisionClass.SignalDeterminationPipeline());

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        ET.reset();

        while (opModeIsActive()) {

            switch (programOrder) {

                case 0:
//                    if (pipeline.type == VisionClass.SignalDeterminationPipeline.SignalSleeveType.LocationONE) {
//                        posOne = true;
//                        posTwo = false;
//                        posThree = false;
//                    }
//                    else if (pipeline.type == VisionClass.SignalDeterminationPipeline.SignalSleeveType.LocationTWO) {
//                        posOne = false;
//                        posTwo = true;
//                        posThree = false;
//                    }
//                    else if (pipeline.type == VisionClass.SignalDeterminationPipeline.SignalSleeveType.LocationTHREE) {
//                        posOne = false;
//                        posTwo = false;
//                        posThree = true;
//                    } else {
//                        posOne = true;
//                        posTwo = false;
//                        posThree = false;
//                    }
//                    programOrder++;
                    break;

                case 1:
//                    SetAttachmentPosition(9520, 0);
//                    if (RailRight.getCurrentPosition() > 9400) {
//                        GyroTurn(90, 0.2);
                    programOrder = 2;
//                    }
                    break;

                case 2:
                    if (MechDrive.GetTaskState() == Task_State.INIT ||
                            MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(-90, 10000, 0.25, 1);
                        programOrder++;
                    }
                    break;

                case 3:
//                    if (MechDrive.GetTaskState() == Task_State.READY ||
//                            MechDrive.GetTaskState() == Task_State.DONE) {
//                        MechDrive.SetTargets(90, 10000, 0.25, 1);
//                        programOrder++;
//                    }
                    break;

                case 4:

                    programOrder++;
                    break;

//                case 10:
//                    if (MechDrive.GetTaskState() == Task_State.INIT ||
//                            MechDrive.GetTaskState() == Task_State.READY ||
//                            MechDrive.GetTaskState() == Task_State.DONE) {
//
//                        if (posOne) {
//                            MechDrive.SetTargets(-90, 800, 0.7, 0);
//                        }
//                        else if (posTwo) {
//                            MechDrive.SetTargets(0, 0, 0.1, 0);
//                        }
//                        else if (posThree) {
//                            MechDrive.SetTargets(90, 800, 0.7, 0);
//                        }
//                        programOrder++;
//                    }
//                    break;

                default:
                    break;
            }

//            rightColorSensorLineDetector();
            leftColorSensorLineDetector();
            centerRightColorSensorLineDetector();
//            centerLeftColorSensorLineDetector();
//            MechDrive.Task(GyroContinuity());
            telemetry.update();
        }

    }

    public void GyroTurn (double angledegree, double power) {

        if (angledegree > GyroContinuity() || turnright) {

            turnright = true;

            if (GyroContinuity() < angledegree) {
                MotorTurn(-power, power, -power, power);
                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();
            } else {
                programOrder++;
                SetMotorPower(0);
                turnright = false;

                while (FrontRight.getCurrentPosition() != 0) {
                    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }
        else if (angledegree < GyroContinuity() || turnleft) {

            turnleft = true;

            if (GyroContinuity() > angledegree) {
                MotorTurn(power, -power, power, -power);
                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();
            } else {
                programOrder++;
                SetMotorPower(0);
                turnleft = false;

                while (FrontRight.getCurrentPosition() != 0) {
                    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

    }

    private double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;
    }

    public void SetMotorPower(double x) {
        FrontLeft.setPower(x);
        FrontRight.setPower(x);
        BackLeft.setPower(x);
        BackRight.setPower(x);
    }

    public void MotorTurn(double FR, double FL, double BR, double BL) {
        FrontRight.setPower(FR);
        FrontLeft.setPower(FL);
        BackRight.setPower(BR);
        BackLeft.setPower(BL);
    }

    public void AttachmentMotorPresets() {
        RailLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RailLeft.setTargetPosition(0);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RailRight.setDirection(DcMotorSimple.Direction.FORWARD);
        RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RailRight.setTargetPosition(0);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RotatingBase.setDirection(DcMotorSimple.Direction.FORWARD);
        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotatingBase.setTargetPosition(0);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SetAttachmentPosition(int railPos, int basePos) {
        RailRight.setTargetPosition(railPos);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailRight.setPower(1);
        RailLeft.setTargetPosition(-railPos);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailLeft.setPower(1);
        RotatingBase.setTargetPosition(basePos);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotatingBase.setPower(1);

    }

    private int rightColorSensorLineDetector() {

        float[] rHSV = new float[3];
        NormalizedRGBA rightRGBA = rightColorsensor.getNormalizedColors();
        rightColorsensor.setGain(30);

        Color.colorToHSV(rightRGBA.toColor(), rHSV);
        telemetry.addData("Right H:", rHSV[0]);
        telemetry.addData("Right S:", rHSV[1]);
        telemetry.addData("Right V:", rHSV[2]);

        int rBlue = 2;
        int rRed = 1;
        int rUnknown = 0;

        //Right Colorsensor from back of Robot
        if (rHSV[0] >= 200 && rHSV[0] <= 230) {
            telemetry.addData("Color:", "Blue");
//            telemetry.update();
            rightBlue = true;
            rightRed = false;
            rightUnknown = false;
            return rBlue;
        } else if (rHSV[0] >= 10 && rHSV[0] <= 50) {
            telemetry.addData("Color:", "Red");
//            telemetry.update();
            rightBlue = false;
            rightRed = true;
            rightUnknown = false;
            return rRed;
        } else {
            telemetry.addData("Color:", "Unknown");
//            telemetry.update();
            rightBlue = false;
            rightRed = false;
            rightUnknown = true;
            return rUnknown;
        }
    }

    private int leftColorSensorLineDetector() {

        float[] lHSV = new float[3];
        NormalizedRGBA leftRGBA = leftColorsensor.getNormalizedColors();
        leftColorsensor.setGain(30);

        Color.colorToHSV(leftRGBA.toColor(), lHSV);
        telemetry.addData("Left H:", lHSV[0]);
        telemetry.addData("Left S:", lHSV[1]);
        telemetry.addData("Left V:", lHSV[2]);

        int lBlue = 2;
        int lRed = 1;
        int lUnknown = 0;

        //Left Colorsensor from back of Robot
        if (lHSV[0] >= 190 && lHSV[0] <= 240) {
            telemetry.addData("Color:", "Blue");
            telemetry.update();
            leftBlue = true;
            leftRed = false;
            leftUnknown = false;
            return lBlue;
        } else if (lHSV[0] >= 10 && lHSV[0] <= 50) {
            telemetry.addData("Color:", "Red");
            telemetry.update();
            leftBlue = false;
            leftRed = true;
            leftUnknown = false;
            return lRed;
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            leftBlue = false;
            leftRed = false;
            leftUnknown = true;
            return lUnknown;
        }
    }

    private int centerRightColorSensorLineDetector() {

        float[] centerRHSV = new float[3];
        NormalizedRGBA centerRightRGBA = centerRightColorsensor.getNormalizedColors();
        centerRightColorsensor.setGain(30);

        Color.colorToHSV(centerRightRGBA.toColor(), centerRHSV);
        telemetry.addData("Center Right H:", centerRHSV[0]);
        telemetry.addData("Center Right S:", centerRHSV[1]);
        telemetry.addData("Center Right V:", centerRHSV[2]);

        int centerRBlue = 2;
        int centerRRed = 1;
        int centerRUnknown = 0;

        //Center Right Colorsensor from back of Robot
        if (centerRHSV[0] >= 190 && centerRHSV[0] <= 240) {
            telemetry.addData("Color:", "Blue");
            telemetry.update();
            centerRightBlue = true;
            centerRightRed = false;
            centerRightUnknown = false;
            return centerRBlue;
        } else if (centerRHSV[0] >= 10 && centerRHSV[0] <= 50) {
            telemetry.addData("Color:", "Red");
            telemetry.update();
            centerRightBlue = false;
            centerRightRed = true;
            centerRightUnknown = false;
            return centerRRed;
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            centerRightBlue = false;
            centerRightRed = false;
            centerRightUnknown = true;
            return centerRUnknown;
        }
    }

    private int centerLeftColorSensorLineDetector() {

        float[] centerLHSV = new float[3];
        NormalizedRGBA centerLeftRGBA = centerLeftColorsensor.getNormalizedColors();
        centerLeftColorsensor.setGain(30);

        Color.colorToHSV(centerLeftRGBA.toColor(), centerLHSV);
        telemetry.addData("Center Left H:", centerLHSV[0]);
        telemetry.addData("Center Left S:", centerLHSV[1]);
        telemetry.addData("Center Left V:", centerLHSV[2]);

        int centerLBlue = 2;
        int centerLRed = 1;
        int centerLUnknown = 0;

        //Center Left Colorsensor from back of Robot
        if (centerLHSV[0] >= 190 && centerLHSV[0] <= 240) {
            telemetry.addData("Color:", "Blue");
            telemetry.update();
            centerLeftBlue = true;
            centerLeftRed = false;
            centerLeftUnknown = false;
            return centerLBlue;
        } else if (centerLHSV[0] >= 10 && centerLHSV[0] <= 50) {
            telemetry.addData("Color:", "Red");
            telemetry.update();
            centerLeftBlue = false;
            centerLeftRed = true;
            centerLeftUnknown = false;
            return centerLRed;
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            centerLeftBlue = false;
            centerLeftRed = false;
            centerLeftUnknown = true;
            return centerLUnknown;
        }
    }

    //From Back of robot
    private void backRightJunctionDetector() {
        if (backRightDistanceSamples_Reset) {
            backRightDistanceSamples[0] = 819;
            backRightDistanceSamples[1] = 819;
            backRightDistanceSamples[2] = 819;
            backRightDistanceSamples_Reset = false;
        }
        else {
            backRightDistanceSamples[backRightCount] = backRightDistanceSensor.getDistance(DistanceUnit.CM);
            backRightCount++;
            backRightDistance = (backRightDistanceSamples[0] + backRightDistanceSamples[1] + backRightDistanceSamples[2]) / 3;

            telemetry.addData("Back Right Distance Sensor", backRightDistance);
            if (backRightDistance < 50) {
                rightBackDistanceCleared = true;
            } else {
                rightBackDistanceCleared = false;
            }

            if (backRightCount > 2) {
                backRightCount = 0;
            }
        }
    }

    //From Back of robot
    private void backLeftJunctionDetector() {
        if (backLeftDistanceSamples_Reset) {
            backLeftDistanceSamples[0] = 122;
            backLeftDistanceSamples[1] = 122;
            backLeftDistanceSamples[2] = 122;
            backLeftDistanceSamples[3] = 122;
            backLeftDistanceSamples[4] = 122;
            backLeftDistanceSamples_Reset = false;
        }
        else {
            backLeftDistanceSamples[backLeftCount] = backLeftDistanceSensor.getDistance(DistanceUnit.CM);
            backLeftCount++;
            backLeftDistance = (backLeftDistanceSamples[0] + backLeftDistanceSamples[1] + backLeftDistanceSamples[2] + backLeftDistanceSamples[3] + backLeftDistanceSamples[4]) / 5;

            telemetry.addData("Back Left Distance Sensor", backLeftDistance);
            if (backLeftDistance < 90) {
                leftBackDistanceCleared = true;
            } else {
                leftBackDistanceCleared = false;
            }

            if (backLeftCount > 4) {
                backLeftCount = 0;
            }
        }
    }

    //From Back of robot
    private void frontRightJunctionDetector() {
        if (frontRightDistanceSamples_Reset) {
            frontRightDistanceSamples[0] = 819;
            frontRightDistanceSamples[1] = 819;
            frontRightDistanceSamples[2] = 819;
//            frontRightDistanceSamples[3] = 819;
//            frontRightDistanceSamples[4] = 819;
            frontRightDistanceSamples_Reset = false;
        }
        else {
//            frontRightDistanceSamples[frontRightCount] = frontRightDistanceSensor.getDistance(DistanceUnit.CM);
            frontRightCount++;
            frontRightDistance = (frontRightDistanceSamples[0] + frontRightDistanceSamples[1] + frontRightDistanceSamples[2]) / 3;

            telemetry.addData("Front Right Distance Sensor", frontRightDistance);
            if (frontRightDistance < 50) {
                rightFrontDistanceCleared = true;
            } else {
                rightFrontDistanceCleared = false;
            }

            if (frontRightCount > 2) {
                frontRightCount = 0;
            }
        }
    }

    //From Back of robot
    private void frontLeftJunctionDetector() {

        if (frontLeftDistanceSamples_Reset) {
            frontLeftDistanceSamples[0] = 819;
            frontLeftDistanceSamples[1] = 819;
            frontLeftDistanceSamples[2] = 819;
//            frontLeftDistanceSamples[3] = 819;
//            frontLeftDistanceSamples[4] = 819;
            frontLeftDistanceSamples_Reset = false;
        }
        else {
//            frontLeftDistanceSamples[frontLeftCount] = frontLeftDistanceSensor.getDistance(DistanceUnit.CM);
            frontLeftCount++;
            frontLeftDistance = (frontLeftDistanceSamples[0] + frontLeftDistanceSamples[1] + frontLeftDistanceSamples[2]) / 3;

            telemetry.addData("Front Left Distance Sensor", frontLeftDistance);
            if (frontLeftDistance < 50) {
                leftFrontDistanceCleared = true;
            } else {
                leftFrontDistanceCleared = false;
            }

            if (frontLeftCount > 2) {
                frontLeftCount = 0;
            }
        }
    }


}

