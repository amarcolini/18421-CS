package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.*
import com.amarcolini.joos.command.CommandScheduler.telem
import com.amarcolini.joos.extensions.getAll
import com.amarcolini.joos.extensions.invoke
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.util.NanoClock
import com.qualcomm.hardware.lynx.LynxModule
import org.opencv.core.Size
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import kotlin.math.roundToInt

class CSRobot : Robot() {
    val drive = Drivetrain(
        MotorGroup(
            hMap, Motor.Type.GOBILDA_312,
            "front_left" to true,
            "back_left" to true,
            "back_right" to false,
            "front_right" to false
        ),
        Motor.Encoder.multiple(
            hMap,
            "front_right" to false, //left odo pod
            "front_left" to true, // right odo pod
            "back_left" to false // perpendicular odo pod
        )
    )

    val verticalExtension = VerticalExtension(
        MotorGroup(
            hMap, Motor.Type.GOBILDA_435,
            "lift_left" to true,
            "lift_right" to false
        ),
        hMap("lift_sensor")
    )

    val intake =
        Intake(
            Motor(hMap, "intake_motor", Motor.Type.GOBILDA_1620),
            Servo(hMap, "intake_servo"),
            hMap("color_left"),
            hMap("color_right"),
        )

    val outtake = Outtake(
        Servo(hMap, "axon"),
        Servo(hMap, "outtake_left"),
        Servo(hMap, "outtake_right"),
    )

    val droneLauncher = DroneLauncher(
        Servo(hMap, "drone_launcher")
    )

    val frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
        hMap("webcam"),
        hMap.appContext.resources
            .getIdentifier("cameraMonitorViewId", "id", hMap.appContext.packageName)
    )

    val frontDistanceSensor = DistanceSensor(hMap("distance_front"))

    val pixelPlopper = PixelPlopper(Servo(hMap, "pixel_claw"))

    private val lynxModules = hMap.getAll<LynxModule>()
    override fun init() {
        register(drive, verticalExtension, intake, outtake, droneLauncher, pixelPlopper)

        verticalExtension.motors.resetEncoders()

        droneLauncher.reset()

        lynxModules.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val clock = NanoClock.system
        var lastTimestamp = clock.seconds()
        register(Component.of {
            lynxModules.forEach {
                it.clearBulkCache()
            }
            val now = clock.seconds()
            telem.addData("Loop hZ", (1.0 / (now - lastTimestamp)).roundToInt())
            telem.addData("numPixels", intake.numPixels)
            lastTimestamp = now
        })
    }

    fun transfer(): Command {
        return SequentialCommand(
            (intake.waitForServoState(Intake.ServoState.UP) and outtake.reset() and {
                intake.motorState = Intake.MotorState.ACTIVE
            }),
            InstantCommand { intake.motorState = Intake.MotorState.STOPPED },
            WaitCommand(0.3),
            outtake.prepareTransfer(),
            WaitCommand(2.0).onInit { intake.motorState = Intake.MotorState.TRANSFER },
            outtake.ready() and WaitCommand(0.3).then(intake.stop()),
        ).requires(intake, outtake, verticalExtension).setInterruptable(false)
            .onEnd {
                intake.motorState = Intake.MotorState.STOPPED
            }
    }

    private fun openCameraAsync(camera: OpenCvCamera, size: Size, onOpen: () -> Unit) {
        camera.openCameraDeviceAsync(
            object : OpenCvCamera.AsyncCameraOpenListener {
                override fun onOpened() {
                    camera.startStreaming(
                        size.width.toInt(),
                        size.height.toInt(),
                        OpenCvCameraRotation.UPRIGHT
                    )
                    onOpen()
                }

                override fun onError(errorCode: Int) {
                    telem.addLine("Camera failed to open!!").setRetained(true)
                    telem.update()
                    stopOpMode()
                }
            })
    }

    fun openFrontCameraAsync(
        useDashboard: Boolean = true,
        size: Size = Size(800.0, 600.0),
        fps: Double = 0.0,
        onOpen: () -> Unit
    ) {
        openCameraAsync(frontCamera, size) {
            if (useDashboard) FtcDashboard.getInstance().startCameraStream(frontCamera, fps)
            onOpen()
        }
    }
}