package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.util.NanoClock
import com.amarcolini.joos.util.deg
import com.qualcomm.hardware.rev.RevColorSensorV3
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

class Intake(
    val motor: Motor,
    val servo: Servo,
    val leftSensor: RevColorSensorV3, val rightSensor: RevColorSensorV3
) :
    AbstractComponent() {
    init {
        motor.runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
        leftSensor.gain = 7f
        rightSensor.gain = 7f
        subcomponents += listOf(motor, servo)
    }

    enum class MotorState(val power: Double) {
        ACTIVE(1.0),
        REVERSE(-0.3),
        STOPPED(0.0),
        TRANSFER(-0.5),
    }

    enum class ServoState(val position: Double) {
        DOWN(0.65), UP(0.09)
    }

    var motorState = MotorState.STOPPED
        set(value) {
            motor.power = value.power
            field = value
        }

    var servoState = ServoState.UP
        set(value) {
            servo.position = value.position
            field = value
        }

    init {
        servo.position = servoState.position
    }

    enum class PixelColor {
        WHITE, GREEN, PURPLE, YELLOW
    }

    private fun getPixelColor(r: Float, g: Float, b: Float): PixelColor? {
        val cmax = maxOf(r, g, b)
        val cmin = minOf(r, g, b)
        val diff = cmax - cmin
        val h = when (cmax) {
            cmin -> 0.0
            r -> (60.0 * ((g - b) / diff) + 360.0) % 360.0
            g -> (60.0 * ((b - r) / diff) + 120.0) % 360.0
            b -> (60.0 * ((b - r) / diff) + 120.0) % 360.0
            else -> 0.0
        }
        val s = if (cmax == 0f) 0.0 else (diff / cmax) * 100.0
        val v = cmax * 100.0
        return when {
            v > 90 -> PixelColor.WHITE
            v < 20 -> null
            h in 110.0..135.0 -> PixelColor.GREEN
            h in 170.0..190.0 -> PixelColor.PURPLE
            h in 70.0..95.0 -> PixelColor.YELLOW
            else -> null
        }
    }

    fun getPixelColors(): Pair<PixelColor?, PixelColor?> {
        val left = leftSensor.normalizedColors
        val right = rightSensor.normalizedColors
        return getPixelColor(left.red, left.green, left.blue) to getPixelColor(right.red, right.green, right.blue)
    }

    fun waitForServoState(state: ServoState): Command {
        return waitForPosition(state.position, 170.deg / servo.range).onEnd {
            servoState = state
        }
            .requires(this)
    }

    var currentTarget = 0.0
    private fun waitForPosition(position: Double, speed: Double): Command = Command.select {
        val correctedPos = position.coerceIn(0.0, 1.0)
        val currentPosition = servo.position
        currentTarget = correctedPos
        Command.of {
            currentTarget = Double.POSITIVE_INFINITY
            servo.position = correctedPos
            currentTarget = servo.position
        }
            .wait(abs(currentPosition - correctedPos) / speed)
            .requires(this)
    }

    fun intake(): Command =
        waitForServoState(ServoState.DOWN).then { motorState = MotorState.ACTIVE }

    fun reverse(): Command =
        waitForServoState(ServoState.DOWN).then { motorState = MotorState.REVERSE }

    fun stop(): Command = Command.of {
        motorState = MotorState.STOPPED
    }.requires(this)

    private var lastUpdateTimestamp: Double? = null
    var numPixels = 0
    override fun update() {
        super.update()
        val now = NanoClock.system.seconds()
        val last = lastUpdateTimestamp
        if (motorState == MotorState.ACTIVE && servoState == ServoState.DOWN) {
            if ((last == null || now - last > 0.1)) {
                numPixels = (if (leftSensor.getDistance(DistanceUnit.MM) < 8.0) 1 else 0) +
                        (if (rightSensor.getDistance(DistanceUnit.MM) < 8.0) 1 else 0)
                lastUpdateTimestamp = now
            }
        } else numPixels = 0
    }
}