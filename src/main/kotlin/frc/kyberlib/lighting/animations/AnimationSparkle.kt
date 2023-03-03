package frc.kyberlib.lighting.animations

import frc.kyberlib.math.randomizer
import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class AnimationSparkle(private val color: Color, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    private var buffer = ArrayList<Color>()

    override fun getBuffer(time: Time, length: Int): List<Color> {
        while(buffer.size < length) { // grow buffer to size
            buffer.add(Color.black)
        }
        while(buffer.size > length) { // shrink buffer to size
            buffer.removeLast()
        }

        for(i in 0 until length) {
            buffer[i] = Color(color.red, color.green, color.blue, randomizer.nextInt(80) + 175)
        }


        return buffer
    }
}