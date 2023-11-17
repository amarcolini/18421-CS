package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.extensions.*
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.extensions.getAll
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.Servo
import com.qualcomm.hardware.lynx.LynxModule
import org.openftc.easyopencv.OpenCvCameraFactory

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
        )
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
        register(drive, verticalExtension, outtake, droneLauncher, pixelPlopper)

        verticalExtension.motors.resetEncoders()

        droneLauncher.reset()

        lynxModules.forEach {
            it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        register(Component.of {
            lynxModules.forEach {
                it.clearBulkCache()
            }
        })
    }
}