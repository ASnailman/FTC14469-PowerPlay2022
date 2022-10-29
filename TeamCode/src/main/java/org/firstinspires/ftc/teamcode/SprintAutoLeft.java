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

@Autonomous(name = "SprintAutoLeft", group = "MecanumDrive")
public class SprintAutoLeft extends LinearOpMode {

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

    //Sensors
    BNO055IMU IMU;
    OpenCvWebcam webcam;
    VisionClassAutoLeft.SignalDeterminationPipeline pipeline;

    static NormalizedColorSensor rightColorsensor;
    static NormalizedColorSensor leftColorsensor;
    static DistanceSensor rightDistanceSensor;
    static DistanceSensor leftDistanceSensor;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;
    Rail_Control RailControl;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    int programOrder = 0;
    ElapsedTime ET = new ElapsedTime();

    double current_value;
    double prev_value = 0;
    double final_value;
    boolean turnright = false;
    boolean turnleft = false;
    boolean posOne;
    boolean posTwo;
    boolean posThree;

    boolean red;
    boolean blue;
    boolean unknown;
    double distance;

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
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");

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
        LeftClaw.setDirection(CRServo.Direction.REVERSE);
        RightClaw.setDirection(CRServo.Direction.FORWARD);
        LeftClaw.setPower(0);
        RightClaw.setPower(0);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
        RailControl = new Rail_Control(RailLeft, RailRight);
        //        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionClassAutoLeft.SignalDeterminationPipeline();
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
                    if (pipeline.type == VisionClassAutoLeft.SignalDeterminationPipeline.SignalSleeveType.LocationONE) {
                        posOne = true;
                        posTwo = false;
                        posThree = false;
                        LeftClaw.setPower(1);
                        RightClaw.setPower(1);
                        ET.reset();
                    } else if (pipeline.type == VisionClassAutoLeft.SignalDeterminationPipeline.SignalSleeveType.LocationTWO) {
                        posOne = false;
                        posTwo = true;
                        posThree = false;
                        LeftClaw.setPower(1);
                        RightClaw.setPower(1);
                        ET.reset();
                    } else if (pipeline.type == VisionClassAutoLeft.SignalDeterminationPipeline.SignalSleeveType.LocationTHREE) {
                        posOne = false;
                        posTwo = false;
                        posThree = true;
                        LeftClaw.setPower(1);
                        RightClaw.setPower(1);
                        ET.reset();
                    } else {
                        posOne = false;
                        posTwo = true;
                        posThree = false;
                        LeftClaw.setPower(1);
                        RightClaw.setPower(1);
                        ET.reset();
                    }
                    programOrder++;
                    break;

                case 1:
                    if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY && ET.milliseconds() > 2000) {
                        SetAttachmentPosition(0, -4903);
                        ET.reset();
                    }
                    else if (RailControl.GetTaskState() == Task_State.DONE) {
                        programOrder++;
                    }
                    break;

                case 2:
                    if (ET.milliseconds() > 900) {
                        if (MechDrive.GetTaskState() == Task_State.INIT ||
                                MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(0, 2350, 0.35, 1);
                                programOrder++;

                        }
                    }
                    break;

                case 3:
                    if (RotatingBase.getCurrentPosition() <= -4700 && RotatingBase.getCurrentPosition() >= -4900) {
                        if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                            SetAttachmentPosition(4610, -4903);
                            programOrder++;
                        }
                    }
                    break;

                case 4:
//                    if (MechDrive.GetTaskState() == Task_State.READY ||
//                            MechDrive.GetTaskState() == Task_State.DONE) {
//                        MechDrive.SetTargets(90, 70, 0.4, 1);
                        programOrder++;
//                    }
                    break;

                case 5:
                    if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                        SetAttachmentPosition(4610, -5644);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 6:
                    if (ET.milliseconds() > 1000) {
                        RightClaw.setPower(-1);
                        LeftClaw.setPower(-1);
                        if (RightClaw.getPower() < -0.95 && LeftClaw.getPower() < -0.95) {
                            ET.reset();
//                            programOrder++;
                            programOrder = 15;
                        }
                    }
                    break;

                case 7:
                    if (ET.milliseconds() > 1000) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(-90, 1080, 0.4, 1);
                            if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                                SetAttachmentPosition(600, 200);
                                ET.reset();
                                programOrder++;
                            }
                        }
                    }
                    break;

                case 8:
                    if (ET.milliseconds() > 1000) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                                MechDrive.SetTargets(-90, 355, 0.4, 1);
                                if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                                    SetAttachmentPosition(590, 200);
                                    ET.reset();
                                    programOrder++;
                                }
                            }
                    }
                    break;

                case 9:
                    if (RotatingBase.getCurrentPosition() > 70 &&
                            ET.milliseconds() > 1000) {
                        RightClaw.setPower(1);
                        LeftClaw.setPower(1);
                        if (RightClaw.getPower() > 0.95 && LeftClaw.getPower() > 0.95) {
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 10:
                    if (ET.milliseconds() > 1000) {
                        if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                            SetAttachmentPosition(4660, 0);
                            programOrder++;
                        }
                    }
                    break;

                case 11:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(92, 1730, 0.4, 1);
                        programOrder++;
                    }
                    break;

                case 12:
                    if (RailRight.getCurrentPosition() > 3600) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(180, 0, 0.4, 1);
                            if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                                SetAttachmentPosition(4660, -3763);
                                ET.reset();
                                programOrder++;
                            }
                        }
                    }
                    break;

                case 13:
                    if (RotatingBase.getCurrentPosition() <= -3650 && ET.milliseconds() > 2000) {
                        RightClaw.setPower(-1);
                        LeftClaw.setPower(-1);
                        if (RightClaw.getPower() < -0.95 && LeftClaw.getPower() < -0.95) {
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 14:
                    if (ET.milliseconds() > 500) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(-90, 850, 0.4, 1);
                            programOrder++;
                        }
                    }
                    break;

                case 15:
                    if (ET.milliseconds() > 800) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {

                            if (posOne) {
                                MechDrive.SetTargets(-90, 1240, 0.4, 1);
                            } else if (posTwo) {
                                MechDrive.SetTargets(90, 100, 0, 1);
                            } else if (posThree) {
                                MechDrive.SetTargets(90, 1310, 0.4, 1);
                            }
                            programOrder++;
                        }
                    }
                    break;

                case 16:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(180, 250, 0.4, 1);
                        if (RailControl.GetTaskState() == Task_State.INIT || RailControl.GetTaskState() == Task_State.READY) {
                            SetAttachmentPosition(0, -3763);
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 17:
                    if (RotatingBase.getCurrentPosition() > -5300) {
                        SetAttachmentPosition(0, -3763);
                        programOrder++;
                    }
                    break;

                default:
                    break;
            }


            MechDrive.Task(GyroContinuity());
            RailControl.RailTask();
            telemetry.addData("backright encoder", BackRight.getCurrentPosition());
            telemetry.addData("gyro", GyroContinuity());
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
        RailControl.SetTargetPosition(railPos, -1, 1);
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

        int Blue = 2;
        int Red = 1;
        int Unknown = 0;

        //Right Colorsensor from front of Robot
        if (rHSV[1] >= 0 && rHSV[1] <= 0.45) {
            telemetry.addData("Color:", "Blue");
//            telemetry.update();
            blue = true;
            red = false;
            unknown = false;
            return Blue;
        } else if (rHSV[1] >= 0.5 && rHSV[1] <= 0.8) {
            telemetry.addData("Color:", "Red");
//            telemetry.update();
            blue = false;
            red = true;
            unknown = false;
            return Red;
        } else {
            telemetry.addData("Color:", "Unknown");
//            telemetry.update();
            blue = false;
            red = false;
            unknown = true;
            return Unknown;
        }
    }

//    private int leftColorSensorLineDetector() {
//
//        float[] lHSV = new float[3];
//        NormalizedRGBA leftRGBA = leftColorsensor.getNormalizedColors();
//        leftColorsensor.setGain(30);
//
//        Color.colorToHSV(leftRGBA.toColor(), lHSV);
//        telemetry.addData("Left H:", lHSV[0]);
//        telemetry.addData("Left S:", lHSV[1]);
//        telemetry.addData("Left V:", lHSV[2]);
//
//        int Blue = 2;
//        int Red = 1;
//        int Unknown = 0;
//
//        //Left Colorsensor from front of Robot
//        if (lHSV[1] >= 0 && lHSV[1] <= 0.45) {
//            telemetry.addData("Color:", "Blue");
//            telemetry.update();
//            blue = true;
//            red = false;
//            unknown = false;
//            return Blue;
//        } else if (lHSV[1] >= 0.5 && lHSV[1] <= 0.8) {
//            telemetry.addData("Color:", "Red");
//            telemetry.update();
//            blue = false;
//            red = true;
//            unknown = false;
//            return Red;
//        } else {
//            telemetry.addData("Color:", "Unknown");
//            telemetry.update();
//            blue = false;
//            red = false;
//            unknown = true;
//            return Unknown;
//        }
//    }

    //From Back of robot
    private void rightWallDetector() {
        distance = rightDistanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Right Distance Sensor", distance);
//        if (distance < 10) {
//            //Motor Power Zero
//            //Increase Case
//            programOrder++;
//        } else {
//            //Motor Power On
//        }
    }

    //From Back of robot
    private void leftWallDetector() {
        distance = leftDistanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Left Distance Sensor", distance);
//        if (distance < 10) {
//            //Motor Power Zero
//            //Increase Case
//            programOrder++;
//        } else {
//            //Motor Power On
//        }
    }

}

