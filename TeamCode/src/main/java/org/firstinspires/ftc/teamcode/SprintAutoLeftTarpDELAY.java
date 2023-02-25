package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "SprintAutoLeftTarpDELAY", group = "MecanumDrive")
public class SprintAutoLeftTarpDELAY extends LinearOpMode {

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
    static DcMotor ExtendingRail;
    static DcMotor RotatingBase;
    static CRServo Claw;
    static CRServo Stopper;
    static ColorSensor rightColorSensor;
    static ColorSensor leftColorSensor;
    static RevBlinkinLedDriver LightStrip;
    static VoltageSensor voltageSensor;

    //Sensors
    BNO055IMU IMU;
    OpenCvWebcam webcam;
    VisionClassAutoLeftBlue.SignalDeterminationPipeline pipeline;

//    static NormalizedColorSensor rightColorsensor;
//    static NormalizedColorSensor leftColorsensor;
//    static NormalizedColorSensor centerRightColorsensor;
//    static NormalizedColorSensor centerLeftColorsensor;
//    static DistanceSensor backRightDistanceSensor;
//    static DistanceSensor backLeftDistanceSensor;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;
    //    Rail_Control RailControl;
    Rail_ControlV2 RailControlV2;
    Direction_Control DirectionControl;
    Base_Control BaseControl;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    int programOrder = 0;
    int repeat = 0;
    ElapsedTime ET = new ElapsedTime();
    ElapsedTime ERT = new ElapsedTime(); //Elapsed Reset Timer
    ElapsedTime EFT = new ElapsedTime(); //Elapsed Failsafe Timer
    int delay;
    int coneReduction;
    int coneLevel = 0;
    int readVoltOnce = 0;
    int angleAdjustment;
    int tickAdjustment;
    int railAdjustment;
    int extendingAdjustment;
    int baseTargetPosition;
    boolean baseMonitorOn = false;
    int railTargetPosition;
    boolean railMonitorOn = false;
    int extendingTargetPosition;
    boolean extendingMonitorOn = false;

    int leftCenterTickCount;

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
    boolean distanceCleared = false;

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

    public void runOpMode() {

        //Initialize the motors and sensors
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Claw = hardwareMap.get(CRServo.class, "Claw");
        Stopper = hardwareMap.get(CRServo.class, "Stopper");
        RailRight = hardwareMap.get(DcMotor.class, "RailRight");
        RailLeft = hardwareMap.get(DcMotor.class, "RailLeft");
        ExtendingRail = hardwareMap.get(DcMotor.class, "ExtendingRail");
        RotatingBase = hardwareMap.get(DcMotor.class, "RotatingBase");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        LightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "LightStrip");

//        rightColorsensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
//        leftColorsensor = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
//        centerRightColorsensor = hardwareMap.get(NormalizedColorSensor.class, "centerRightColorSensor");
//        centerLeftColorsensor = hardwareMap.get(NormalizedColorSensor.class, "centerLeftColorSensor");
//        backRightDistanceSensor = hardwareMap.get(DistanceSensor.class, "backRightDistanceSensor");
//        backLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backLeftDistanceSensor");

//        voltageSensor = hardwareMap.get(VoltageSensor.class, "VoltageSensor");

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
        Claw.setDirection(CRServo.Direction.FORWARD);
        Claw.setPower(0);

        //Stopper Presets
        Stopper.setDirection(DcMotorSimple.Direction.FORWARD);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
//        RailControl = new Rail_Control(RailLeft, RailRight);
        RailControlV2 = new Rail_ControlV2(RailLeft, RailRight);
        //        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);
        DirectionControl = new Direction_Control(IMU, FrontLeft, FrontRight, BackLeft, BackRight);
        BaseControl = new Base_Control(RotatingBase);

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionClassAutoLeftBlue.SignalDeterminationPipeline();
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

        //delay
        delay = 0;
        //cone reduction
        coneReduction = 0;

        waitForStart();

        ET.reset();
        ERT.reset();

        if (readVoltOnce == 0) {
            telemetry.addData("voltage", "%.1f volts", new Func<Double>() { @Override public Double value() { return getBatteryVoltage(); } });
            if (getBatteryVoltage() > 13.7) {
                angleAdjustment = 22;
                tickAdjustment = -5;
                railAdjustment = -20;
            }
            else if (getBatteryVoltage() > 13.2) {
                angleAdjustment = 12;
                tickAdjustment = 5;
                railAdjustment = 0;
            }
            else if (getBatteryVoltage() > 12.7) {
                angleAdjustment = -5;
                tickAdjustment = 5;
                railAdjustment = 10;
            }
            else {
                angleAdjustment = -5;
                tickAdjustment = 5;
                railAdjustment = 40;
            }
            readVoltOnce++;

        }
        telemetry.update();

        while (opModeIsActive()) {

            LightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

            switch (programOrder) {

                case 0:
                    if (pipeline.type == VisionClassAutoLeftBlue.SignalDeterminationPipeline.SignalSleeveType.LocationONE) {
                        posOne = true;
                        posTwo = false;
                        posThree = false;
                        Claw.setPower(1);
                        Stopper.setPower(-0.4);
                        ET.reset();
                        ERT.reset();
                    } else if (pipeline.type == VisionClassAutoLeftBlue.SignalDeterminationPipeline.SignalSleeveType.LocationTWO) {
                        posOne = false;
                        posTwo = true;
                        posThree = false;
                        Claw.setPower(1);
                        Stopper.setPower(-0.4);
                        ET.reset();
                        ERT.reset();
                    } else if (pipeline.type == VisionClassAutoLeftBlue.SignalDeterminationPipeline.SignalSleeveType.LocationTHREE) {
                        posOne = false;
                        posTwo = false;
                        posThree = true;
                        Claw.setPower(1);
                        Stopper.setPower(-0.4);
                        ET.reset();
                        ERT.reset();
                    } else {
                        posOne = false;
                        posTwo = true;
                        posThree = false;
                        Claw.setPower(1);
                        Stopper.setPower(-0.4);
                        ET.reset();
                        ERT.reset();
                    }
                    programOrder++;
                    break;

                case 1:
                    if (ET.milliseconds() > 550 + delay) {
                        if (RailControlV2.GetTaskState() == Task_State.INIT || RailControlV2.GetTaskState() == Task_State.READY) {
                            SetAttachment_LowPwr2Rail(3025 + railAdjustment, -1220);
//                            SetAttachment_LowPwr2Rail(2970, 1540 + angleAdjustment);
                            MechDrive.SetTargets(0, 2160, 0.7, 1);
                            programOrder++;
                        }
                    }
                    break;

                case 2:
                    ET.reset();
                    programOrder++;
                    break;

                case 3:
                    if (ET.milliseconds() > 700) {
                        Stopper.setPower(0.6);
                        programOrder++;
                    }
                    break;

                case 4:
                    programOrder++;
                    break;

                case 5:
                    if (RotatingBase.getCurrentPosition() <= -1170 && RotatingBase.getCurrentPosition() >= -1270) {
                        SetExtendingPosition(100 + tickAdjustment);
                        programOrder++;
                    }
                    break;

                case 6:
                    if (MechDrive.GetTaskState() == Task_State.DONE || MechDrive.GetTaskState() == Task_State.READY) {
                        DirectionControl.SetTargetDirection(0, 0.2);
                        SetAttachmentPositionLowPower(3025 + railAdjustment, -1680 + angleAdjustment);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 7:
//                        if (RotatingBase.getCurrentPosition() >= 1520 + angleAdjustment && (RailControlV2.GetTaskState() == Task_State.DONE ||
//                                RailControlV2.GetTaskState() == Task_State.READY)) {
                    if (BaseControl.GetTaskState() == Task_State.READY || BaseControl.GetTaskState() == Task_State.DONE) {
                        if (coneLevel == 0) {
                            if (ET.milliseconds() > 250) {
                                SetAttachment_LowPwrRail(2690, -1680 + angleAdjustment);
                                ET.reset();
                                programOrder++;
                            }
                        }
                        else {
                            if (ET.milliseconds() > 250) {
                                SetAttachment_LowPwrRail(2690, -1602 + angleAdjustment);
                                //NO TARP
//                                SetAttachment_LowPwrRail(2690, -1542 + angleAdjustment);
                                ET.reset();
                                programOrder++;
                            }
                        }
                    }
                    break;

                case 8:
                    if (ET.milliseconds() > 200) {
                        Claw.setPower(-0.4);
                        if (Claw.getPower() < -0.3) {
                            programOrder++;
                            ET.reset();
                        }
                    }
                    break;

                case 9:
                    if (coneLevel == 5 - coneReduction) {
                        Stopper.setPower(-0.8);
                        ET.reset();
                        programOrder = 17;
                    } else {
                        programOrder++;
                        ET.reset();
                    }
                    break;

                case 10:
                    if (ET.milliseconds() > 200) {
                        SetAttachmentPosition(2690, 0);
                        Stopper.setPower(-0.8);
                        SetExtendingPositionLowPower(0);
                        programOrder++;
                    }
                    break;

                case 11:
                    if (RotatingBase.getCurrentPosition() > -1020) {
                        if (coneLevel == 0) {
                            SetAttachmentPosition(570, -30);
                        }
                        else if (coneLevel == 1) {
                            SetAttachmentPosition(490, -30);
                        }
                        else if (coneLevel == 2) {
                            SetAttachmentPosition(380, -30);
                        }
                        else if (coneLevel == 3) {
                            SetAttachmentPosition(295, -30);
                        }
                        else if (coneLevel == 4) {
                            SetAttachmentPosition(180, -30);
                        }
                        programOrder++;
                    }
                    break;

                case 12:
                    if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
                        SetExtendingPosition(550 + tickAdjustment);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 13:
                    if (ET.milliseconds() > 330) {
                        Claw.setPower(1);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 14:
                    if (ET.milliseconds() > 450) {
                        SetAttachmentPosition(1200, 0);
                        programOrder++;
                    }
                    break;

                case 15:
                    if (RailControlV2.GetTaskState() == Task_State.DONE || RailControlV2.GetTaskState() == Task_State.READY) {
                        SetExtendingPositionLowPower(120);
                        Stopper.setPower(0.6);
//                        SetExtendingPositionLowPower(92 + tickAdjustment);
                        SetAttachmentPositionLowPower(2985 + railAdjustment, -1602 + angleAdjustment);
                        //NO TARP
//                        SetAttachmentPositionLowPower(3055, -1542 + angleAdjustment);
                        ET.reset();
                        programOrder++;
                    }
                    break;

                case 16:
                    if (ET.milliseconds() > 100) {
                        ET.reset();
                        coneLevel++;
                        if (coneLevel < 6 - coneReduction) {
                            programOrder = 7;
                        }
                    }
                    break;

                case 17:
//                    if (ET.milliseconds() > 200) {
//                        SetAttachmentPosition(0,0);
//                        MechDrive.SetTargets(0, 50, 0.3, 1);
                    if (ET.milliseconds() > 300) {
                        DirectionControl.Override();
                        programOrder++;
                    }
//                    }
                    break;

                case 18:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {

                        if (posOne) {
                            MechDrive.SetTargets(-90, 1250, 0.8, 1);
                            SetAttachmentPosition(0, -1020);
                            SetExtendingPosition(0);
                            ET.reset();
                        } else if (posTwo) {
                            MechDrive.SetTargets(-90, 0, 0, 1);
                            SetAttachmentPosition(0, -1020);
                            SetExtendingPosition(0);
                            ET.reset();
                        } else if (posThree) {
                            MechDrive.SetTargets(88, 3070, 0.8, 1);
                            SetAttachmentPosition(0, -200);
                            SetExtendingPosition(0);
                            ET.reset();
                        }
                        programOrder++;
                    }
                    break;

                case 19:
                    if (MechDrive.GetTaskState() == Task_State.READY ||
                            MechDrive.GetTaskState() == Task_State.DONE) {
                        if (posOne || posTwo) {
                            SetAttachmentPosition(0, -1020);
                            MechDrive.SetTargets(180, 300, 0.6, 1);
                        }
//                        else if (posThree) {
//                            MechDrive.SetTargets(180, 100, 0.6, 1);
//                        }
                        DirectionControl.SetTargetDirection(0, 0.2);
                        programOrder++;
                    }
                    break;

                default:
                    break;
            }

//            if (ERT.milliseconds() > 29500) {
//                SetAttachmentPosition(0, -1020);
//            }

            if (baseMonitorOn) {
                if (RotatingBase.getCurrentPosition() != baseTargetPosition) {
                    if (EFT.milliseconds() > 700) {
                        SetAttachmentPosition(0, 1020);
                    }
                }
            }
            if (extendingMonitorOn) {
                if (ExtendingRail.getCurrentPosition() != extendingTargetPosition) {
                    if (EFT.milliseconds() > 700) {
                        SetAttachmentPosition(0, 1020);
                    }
                }
            }
            if (railMonitorOn) {
                if (RailRight.getCurrentPosition() != railTargetPosition) {
                    if (EFT.milliseconds() > 700) {
                        SetAttachmentPosition(0, 1020);
                    }
                }
            }

//            rightColorSensorLineDetector();
//            leftColorSensorLineDetector();
//            centerRightColorSensorLineDetector();
//            centerLeftColorSensorLineDetector();
//            backRightJunctionDetector();
//            backLeftJunctionDetector();
//            frontRightJunctionDetector();
//            frontLeftJunctionDetector();
            DirectionControl.GyroTask();
            MechDrive.Task(GyroContinuity());
//            RailControl.RailTask();
            RailControlV2.RailTask();
            BaseControl.RotatingBaseTask();
//            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.addData("LeftCenterTicks", leftCenterTickCount);
//            telemetry.addData("backright encoder", BackRight.getCurrentPosition());
            telemetry.addData("ExtendingRail", ExtendingRail.getCurrentPosition());
            telemetry.addData("gyro", GyroContinuity());
            telemetry.update();
        }

    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
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

        RotatingBase.setDirection(DcMotorSimple.Direction.REVERSE);
        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotatingBase.setTargetPosition(0);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ExtendingRail.setDirection(DcMotorSimple.Direction.FORWARD);
        ExtendingRail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendingRail.setTargetPosition(0);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SetAttachment_LowPwr2Rail(int railPos, int basePos) {
        RailControlV2.SetTargetPosition(railPos, -0.95, 0.95);
//        RotatingBase.setTargetPosition(basePos);
//        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RotatingBase.setPower(1);
        BaseControl.SetTargetPosition(basePos, -0.3, 0.3);
    }

    public void SetAttachment_LowPwrRail(int railPos, int basePos) {
        RailControlV2.SetTargetPosition(railPos, -0.5, 0.5);
//        RotatingBase.setTargetPosition(basePos);
//        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RotatingBase.setPower(1);
        BaseControl.SetTargetPosition(basePos, -1, 1);
    }

    public void SetAttachmentPosition(int railPos, int basePos) {
        RailControlV2.SetTargetPosition(railPos, -1, 1);
//        RotatingBase.setTargetPosition(basePos);
//        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RotatingBase.setPower(1);
        BaseControl.SetTargetPosition(basePos, -1, 1);
    }

    public void SetExtendingPosition(int extendingPos) {
        ExtendingRail.setTargetPosition(extendingPos);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendingRail.setPower(1);
    }

    public void SetExtendingPositionLowPower(int extendingPos) {
        ExtendingRail.setTargetPosition(extendingPos);
        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendingRail.setPower(0.6);
    }

    public void SetAttachmentPositionLowPower(int railPos, int basePos) {
        RailControlV2.SetTargetPosition(railPos, -1, 1);
//        RotatingBase.setTargetPosition(basePos);
//        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RotatingBase.setPower(0.5);
        BaseControl.SetTargetPosition(basePos, -0.5, 0.5);
    }

//    private int rightColorSensorLineDetector() {
//
//        float[] rHSV = new float[3];
//        NormalizedRGBA rightRGBA = rightColorsensor.getNormalizedColors();
//        rightColorsensor.setGain(30);
//
//        Color.colorToHSV(rightRGBA.toColor(), rHSV);
//        telemetry.addData("Right H:", rHSV[0]);
//        telemetry.addData("Right S:", rHSV[1]);
//        telemetry.addData("Right V:", rHSV[2]);
//
//        int rBlue = 2;
//        int rRed = 1;
//        int rUnknown = 0;
//
//        //Right Colorsensor from back of Robot
//        if (rHSV[0] >= 170 && rHSV[0] <= 240) {
//            telemetry.addData("Color:", "Blue");
////            telemetry.update();
//            rightBlue = true;
//            rightRed = false;
//            rightUnknown = false;
//            return rBlue;
//        } else if (rHSV[0] >= 10 && rHSV[0] <= 50) {
//            telemetry.addData("Color:", "Red");
////            telemetry.update();
//            rightBlue = false;
//            rightRed = true;
//            rightUnknown = false;
//            return rRed;
//        } else {
//            telemetry.addData("Color:", "Unknown");
////            telemetry.update();
//            rightBlue = false;
//            rightRed = false;
//            rightUnknown = true;
//            return rUnknown;
//        }
//    }
//
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
//        int lBlue = 2;
//        int lRed = 1;
//        int lUnknown = 0;
//
//        //Left Colorsensor from back of Robot
//        if (lHSV[0] >= 170 && lHSV[0] <= 240) {
//            telemetry.addData("Color:", "Blue");
//            telemetry.update();
//            leftBlue = true;
//            leftRed = false;
//            leftUnknown = false;
//            return lBlue;
//        } else if (lHSV[0] >= 10 && lHSV[0] <= 50) {
//            telemetry.addData("Color:", "Red");
//            telemetry.update();
//            leftBlue = false;
//            leftRed = true;
//            leftUnknown = false;
//            return lRed;
//        } else {
//            telemetry.addData("Color:", "Unknown");
//            telemetry.update();
//            leftBlue = false;
//            leftRed = false;
//            leftUnknown = true;
//            return lUnknown;
//        }
//    }
//
//    private int centerRightColorSensorLineDetector() {
//
//        float[] centerRHSV = new float[3];
//        NormalizedRGBA centerRightRGBA = centerRightColorsensor.getNormalizedColors();
//        centerRightColorsensor.setGain(30);
//
//        Color.colorToHSV(centerRightRGBA.toColor(), centerRHSV);
//        telemetry.addData("Center Right H:", centerRHSV[0]);
//        telemetry.addData("Center Right S:", centerRHSV[1]);
//        telemetry.addData("Center Right V:", centerRHSV[2]);
//
//        int centerRBlue = 2;
//        int centerRRed = 1;
//        int centerRUnknown = 0;
//
//        //Center Right Colorsensor from back of Robot
//        if (centerRHSV[0] >= 170 && centerRHSV[0] <= 240) {
//            telemetry.addData("Color:", "Blue");
//            telemetry.update();
//            centerRightBlue = true;
//            centerRightRed = false;
//            centerRightUnknown = false;
//            return centerRBlue;
//        } else if (centerRHSV[0] >= 10 && centerRHSV[0] <= 50) {
//            telemetry.addData("Color:", "Red");
//            telemetry.update();
//            centerRightBlue = false;
//            centerRightRed = true;
//            centerRightUnknown = false;
//            return centerRRed;
//        } else {
//            telemetry.addData("Color:", "Unknown");
//            telemetry.update();
//            centerRightBlue = false;
//            centerRightRed = false;
//            centerRightUnknown = true;
//            return centerRUnknown;
//        }
//    }
//
//    private int centerLeftColorSensorLineDetector() {
//
//        float[] centerLHSV = new float[3];
//        NormalizedRGBA centerLeftRGBA = centerLeftColorsensor.getNormalizedColors();
//        centerLeftColorsensor.setGain(30);
//
//        Color.colorToHSV(centerLeftRGBA.toColor(), centerLHSV);
//        telemetry.addData("Center Left H:", centerLHSV[0]);
//        telemetry.addData("Center Left S:", centerLHSV[1]);
//        telemetry.addData("Center Left V:", centerLHSV[2]);
//
//        int centerLBlue = 2;
//        int centerLRed = 1;
//        int centerLUnknown = 0;
//
//        //Center Left Colorsensor from back of Robot
//        if (centerLHSV[0] >= 170 && centerLHSV[0] <= 240) {
//            telemetry.addData("Color:", "Blue");
//            telemetry.update();
//            centerLeftBlue = true;
//            centerLeftRed = false;
//            centerLeftUnknown = false;
//            return centerLBlue;
//        } else if (centerLHSV[0] >= 10 && centerLHSV[0] <= 50) {
//            telemetry.addData("Color:", "Red");
//            telemetry.update();
//            centerLeftBlue = false;
//            centerLeftRed = true;
//            centerLeftUnknown = false;
//            return centerLRed;
//        } else {
//            telemetry.addData("Color:", "Unknown");
//            telemetry.update();
//            centerLeftBlue = false;
//            centerLeftRed = false;
//            centerLeftUnknown = true;
//            return centerLUnknown;
//        }
//    }
//
//    //From Back of robot
//    private void backRightJunctionDetector() {
//        if (backRightDistanceSamples_Reset) {
//            backRightDistanceSamples[0] = 819;
//            backRightDistanceSamples[1] = 819;
//            backRightDistanceSamples[2] = 819;
//            backRightDistanceSamples_Reset = false;
//        }
//        else {
//            backRightDistanceSamples[backRightCount] = backRightDistanceSensor.getDistance(DistanceUnit.CM);
//            backRightCount++;
//            backRightDistance = (backRightDistanceSamples[0] + backRightDistanceSamples[1] + backRightDistanceSamples[2]) / 3;
//
//            telemetry.addData("Back Right Distance Sensor", backRightDistance);
//            if (backRightDistance < 50) {
//                rightBackDistanceCleared = true;
//            } else {
//                rightBackDistanceCleared = false;
//            }
//
//            if (backRightCount > 2) {
//                backRightCount = 0;
//            }
//        }
//    }
//
//    //From Back of robot
//    private void backLeftJunctionDetector() {
//        if (backLeftDistanceSamples_Reset) {
//            backLeftDistanceSamples[0] = 122;
//            backLeftDistanceSamples[1] = 122;
//            backLeftDistanceSamples[2] = 122;
//            backLeftDistanceSamples[3] = 122;
//            backLeftDistanceSamples[4] = 122;
//            backLeftDistanceSamples_Reset = false;
//        }
//        else {
//            backLeftDistanceSamples[backLeftCount] = backLeftDistanceSensor.getDistance(DistanceUnit.CM);
//            backLeftCount++;
//            backLeftDistance = (backLeftDistanceSamples[0] + backLeftDistanceSamples[1] + backLeftDistanceSamples[2] + backLeftDistanceSamples[3] + backLeftDistanceSamples[4]) / 5;
//
//            telemetry.addData("Back Left Distance Sensor", backLeftDistance);
//            if (backLeftDistance < 90) {
//                leftBackDistanceCleared = true;
//            } else {
//                leftBackDistanceCleared = false;
//            }
//
//            if (backLeftCount > 4) {
//                backLeftCount = 0;
//            }
//        }
//    }

//    //From Back of robot
//    private void frontRightJunctionDetector() {
//        if (frontRightDistanceSamples_Reset) {
//            frontRightDistanceSamples[0] = 819;
//            frontRightDistanceSamples[1] = 819;
//            frontRightDistanceSamples[2] = 819;
////            frontRightDistanceSamples[3] = 819;
////            frontRightDistanceSamples[4] = 819;
//            frontRightDistanceSamples_Reset = false;
//        }
//        else {
////            frontRightDistanceSamples[frontRightCount] = frontRightDistanceSensor.getDistance(DistanceUnit.CM);
//            frontRightCount++;
//            frontRightDistance = (frontRightDistanceSamples[0] + frontRightDistanceSamples[1] + frontRightDistanceSamples[2]) / 3;
//
//            telemetry.addData("Front Right Distance Sensor", frontRightDistance);
//            if (frontRightDistance < 50) {
//                rightFrontDistanceCleared = true;
//            } else {
//                rightFrontDistanceCleared = false;
//            }
//
//            if (frontRightCount > 2) {
//                frontRightCount = 0;
//            }
//        }
//    }

//    //From Back of robot
//    private void frontLeftJunctionDetector() {
//
//        if (frontLeftDistanceSamples_Reset) {
//            frontLeftDistanceSamples[0] = 819;
//            frontLeftDistanceSamples[1] = 819;
//            frontLeftDistanceSamples[2] = 819;
////            frontLeftDistanceSamples[3] = 819;
////            frontLeftDistanceSamples[4] = 819;
//            frontLeftDistanceSamples_Reset = false;
//        }
//        else {
////            frontLeftDistanceSamples[frontLeftCount] = frontLeftDistanceSensor.getDistance(DistanceUnit.CM);
//            frontLeftCount++;
//            frontLeftDistance = (frontLeftDistanceSamples[0] + frontLeftDistanceSamples[1] + frontLeftDistanceSamples[2]) / 3;
//
//            telemetry.addData("Front Left Distance Sensor", frontLeftDistance);
//            if (frontLeftDistance < 50) {
//                leftFrontDistanceCleared = true;
//            } else {
//                leftFrontDistanceCleared = false;
//            }
//
//            if (frontLeftCount > 2) {
//                frontLeftCount = 0;
//            }
//        }
//    }

}