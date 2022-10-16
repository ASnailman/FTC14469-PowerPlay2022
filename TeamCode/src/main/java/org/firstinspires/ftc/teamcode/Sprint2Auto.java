package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Sprint2Auto", group = "MecanumDrive")
public class Sprint2Auto extends LinearOpMode {

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
    static Servo Claw;

    //Sensors
    BNO055IMU IMU;
    OpenCvWebcam webcam;
    VisionClass.SignalDeterminationPipeline pipeline;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;

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

    public void runOpMode() {

        //Initialize the motors and sensors
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Claw = hardwareMap.get(Servo.class, "Claw");
        RailRight = hardwareMap.get(DcMotor.class, "RailRight");
        RailLeft = hardwareMap.get(DcMotor.class, "RailLeft");
        RotatingBase = hardwareMap.get(DcMotor.class, "RotatingBase");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

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
        Claw.setDirection(Servo.Direction.FORWARD);
        Claw.scaleRange(0, 1);
        Claw.setPosition(1);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
//        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionClass.SignalDeterminationPipeline();
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
                    if (pipeline.type == VisionClass.SignalDeterminationPipeline.SignalSleeveType.LocationONE) {
                        posOne = true;
                        posTwo = false;
                        posThree = false;
                    } else if (pipeline.type == VisionClass.SignalDeterminationPipeline.SignalSleeveType.LocationTWO) {
                        posOne = false;
                        posTwo = true;
                        posThree = false;
                    } else if (pipeline.type == VisionClass.SignalDeterminationPipeline.SignalSleeveType.LocationTHREE) {
                        posOne = false;
                        posTwo = false;
                        posThree = true;
                    } else {
                        posOne = true;
                        posTwo = false;
                        posThree = false;
                    }
                    programOrder++;
                    break;

                case 1:
                    SetAttachmentPosition(150, 7526);
                    ET.reset();
                    programOrder++;
                    break;

                case 2:
                    if (ET.milliseconds() > 900) {
                        if (MechDrive.GetTaskState() == Task_State.INIT ||
                                MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(0, 2120, 0.4, 1);
                                programOrder++;

                        }
                    }
                    break;

                case 3:
                    if (RotatingBase.getCurrentPosition() >= 7496 && RotatingBase.getCurrentPosition() <= 7556) {
                        SetAttachmentPosition(4660, 7526);
                        if (RailRight.getCurrentPosition() >= 2700) {
                            programOrder++;
                        }
                    }
                    break;

                case 4:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(-90, 540, 0.4, 1);
                        programOrder++;
                    }
                    break;

                case 5:
//                    if (ET.milliseconds() > 500) {
//                        if (MechDrive.GetTaskState() == Task_State.READY ||
//                                MechDrive.GetTaskState() == Task_State.DONE) {
//                            MechDrive.SetTargets(0, 150, 0.4, 1);
//                            ET.reset();
                    SetAttachmentPosition(4660, 3763);
                    programOrder++;
//                        }
//                    }
                    break;

                case 6:
                    if ( (MechDrive.GetTaskState() == Task_State.READY || MechDrive.GetTaskState() == Task_State.DONE)
                            && (RailRight.getCurrentPosition() >= 4630 && RailRight.getCurrentPosition() <= 4690)
                            && (RotatingBase.getCurrentPosition() >= 3733 && RotatingBase.getCurrentPosition() <= 3793)
                    ) {
                        Claw.setPosition(0);
                        if (Claw.getPosition() < 0.05) {
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 7:
                    if (ET.milliseconds() > 500) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(90, 1450, 0.4, 1);
                            SetAttachmentPosition(600, -100);
                            programOrder++;
                        }
                    }
                    break;

                case 8:
                    if (RailRight.getCurrentPosition() <= 700) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(90, 455, 0.4, 1);
                            SetAttachmentPosition(570, -100);
                            ET.reset();
                            programOrder++;
                            }
                        }
                    break;

                case 9:
                    if (RailRight.getCurrentPosition() > 470 &&
                            RotatingBase.getCurrentPosition() < -70 &&
                            ET.milliseconds() > 1000) {
                        Claw.setPosition(1);
                        if (Claw.getPosition() > 0.95) {
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 10:
                    if (ET.milliseconds() > 1000) {
                        SetAttachmentPosition(4660, 0);
                        if (RailRight.getCurrentPosition() >= 1600) {
                            programOrder++;
                        }
                    }
                    break;

                case 11:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(-92, 1730, 0.4, 1);
                        programOrder++;
                    }
                    break;

                case 12:
                    if (RailRight.getCurrentPosition() > 3600) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            SetAttachmentPosition(4660, 3763);
                            MechDrive.SetTargets(180, 0, 0.4, 1);
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 13:
                    if (RotatingBase.getCurrentPosition() >= 3700 && ET.milliseconds() > 2000) {
                        Claw.setPosition(0);
                        if (Claw.getPosition() < 0.05) {
                            ET.reset();
                            programOrder++;
                        }
                    }
                    break;

                case 14:
                    if (ET.milliseconds() > 500) {
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {
                            MechDrive.SetTargets(90, 850, 0.4, 1);
                            programOrder++;
                        }
                    }
                    break;

                case 15:
                        if (MechDrive.GetTaskState() == Task_State.READY ||
                                MechDrive.GetTaskState() == Task_State.DONE) {

                            if (posOne) {
                                MechDrive.SetTargets(-90, 800, 0.4, 1);
                            } else if (posTwo) {
                                MechDrive.SetTargets(90, 0, 0, 1);
                            } else if (posThree) {
                                MechDrive.SetTargets(90, 2000, 0.4, 1);
                            }
                            programOrder++;
                        }
                    break;

                case 16:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        MechDrive.SetTargets(180, 250, 0.4, 1);
                        SetAttachmentPosition(0, 3763);
                        programOrder++;
                    }
                    break;

                default:
                    break;
            }

            MechDrive.Task(GyroContinuity());
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
        RailRight.setTargetPosition(-railPos);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailRight.setPower(1);
        RailLeft.setTargetPosition(railPos);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailLeft.setPower(1);
        RotatingBase.setTargetPosition(basePos);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotatingBase.setPower(1);

    }

}

