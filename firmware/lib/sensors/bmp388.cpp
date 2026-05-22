#include "bmp388.h"

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

namespace {
Adafruit_BMP3XX bmp;
float groundPressure_hPa = 1013.25f;  // sea-level default until init() calibrates

// Standard barometric formula, referenced to the captured ground pressure
// so the result is altitude above the launch pad.
float pressureToAltitude(float pressure_hPa) {
    return 44330.0f * (1.0f - powf(pressure_hPa / groundPressure_hPa, 0.1903f));
}
}  // namespace

bool bmp388::init(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);

    // GY-BMP388 modules ship strapped to either 0x77 or 0x76 — try both.
    if (!bmp.begin_I2C(0x77, &Wire) && !bmp.begin_I2C(0x76, &Wire)) {
        return false;
    }

    // Drone-style config: heavy pressure oversampling plus the IIR filter
    // for low-noise altitude at the 50 Hz control-loop rate.
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Capture the launch-pad pressure as the 0 m reference by averaging a
    // short burst of readings to suppress per-sample noise.
    float sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 20; i++) {
        if (bmp.performReading()) {
            sum += bmp.pressure / 100.0f;  // Pa -> hPa
            samples++;
        }
        delay(20);
    }
    if (samples == 0) {
        return false;
    }
    groundPressure_hPa = sum / samples;
    return true;
}

float bmp388::readAltitude() {
    if (!bmp.performReading()) {
        return NAN;
    }
    return pressureToAltitude(bmp.pressure / 100.0f);  // bmp.pressure is in Pa
}
