package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.command.CommandScheduler.telem
import com.amarcolini.joos.extensions.getAll
import com.amarcolini.joos.extensions.invoke
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.util.NanoClock
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.opmode.TransferTest
import org.openftc.easyopencv.OpenCvCameraFactory
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

    val webcam = OpenCvCameraFactory.getInstance().createWebcam(
        hMap("webcam"),
        hMap.appContext.resources
            .getIdentifier("cameraMonitorViewId", "id", hMap.appContext.packageName)
    )

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
            WaitCommand(1.5).onInit { intake.motorState = Intake.MotorState.REVERSE },
            outtake.ready() and WaitCommand(0.3).then(intake.stop()),
        ).requires(intake, outtake).setInterruptable(false)
    }
}