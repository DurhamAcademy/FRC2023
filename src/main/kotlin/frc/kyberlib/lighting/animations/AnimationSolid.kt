package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import java.awt.Color

class AnimationSolid(val color: Color, enableTransparency: Boolean = false, condition: () -> Boolean = { true }) :
    LEDAnimation(condition, enableTransparency) {

    private var buf: MutableList<Color> = MutableList<Color>(0) { color }
    override fun getBuffer(time: Time, length: Int): List<Color> {
        if (buf.size != length) buf = MutableList<Color>(length) { color }

        return buf
    }
}
