#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

static constexpr float MAX_SPEED = 255.0f;

enum WheelMapping {
    TOP_LEFT = 1,
    TOP_RIGHT = 0,
    BOTTOM_LEFT = 2,
    BOTTOM_RIGHT = 3,
};

static float vw = 0;
static float vx = 0;
static float vy = 0;
uint8_t speed_divider = 2;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

static inline uint8_t clamp_to_uint8(float val) {
    return val < 0 ? 0 : val > 255 ? 255 : static_cast<uint8_t>(val);
}

static void set_motor_speed(int motor_index, float motor_speed) {
    Adafruit_DCMotor* motor = AFMS.getMotor(motor_index + 1);
    if (motor_speed < 0) {
        motor->run(BACKWARD);
        motor->setSpeed(max(0, min(MAX_SPEED, -motor_speed)));
    } else {
        motor->run(FORWARD);
        motor->setSpeed(max(0, min(MAX_SPEED, motor_speed)));
    }
}

static void update_speed() {
    float wheel_speeds[4];
    wheel_speeds[TOP_LEFT] = vx + vy - vw;
    wheel_speeds[TOP_RIGHT] = vx - vy + vw;
    wheel_speeds[BOTTOM_LEFT] = vx - vy - vw;
    wheel_speeds[BOTTOM_RIGHT] = vx + vy + vw;
    float max_wheel_speed = 0;
    for (int i = 0; i < 4; ++i) {
        max_wheel_speed = max(max_wheel_speed, abs(wheel_speeds[i]));
    }
    for (int i = 0; i < 4; ++i) {
        float scale = min(1, MAX_SPEED / max_wheel_speed);
        set_motor_speed(i, wheel_speeds[i] * scale);
    }
}

void setup() {
    Serial.begin(9600);
    Dabble.begin(9600);
    AFMS.begin();
    Serial.println("Program Reset");
}

void loop() {
    Dabble.processInput();
    vx = GamePad.isUpPressed() ? 2 : GamePad.isDownPressed() ? -2 : GamePad.getYaxisData() / 4;
    vy = GamePad.isRightPressed() ? 2 : GamePad.isLeftPressed() ? -2 : GamePad.getXaxisData() / 4;

    if (GamePad.isCrossPressed()) {
        vw = 1;
    } else if (GamePad.isCirclePressed()) {
        vw = -1;
    } else if (GamePad.isSelectPressed()) {
        vw = -vx;
        vx = 0;
    } else if (GamePad.isStartPressed()) {
        vw = -vy;
        vy = 0;
    } else {
        vw = 0;
    }

    static bool holding_button = false;
    if (GamePad.isSquarePressed()) {
        if (!holding_button && speed_divider < 5) {
            ++speed_divider;
        }
        holding_button = true;
    } else if (GamePad.isTrianglePressed()) {
        if (!holding_button && speed_divider > 1) {
            --speed_divider;
        }
        holding_button = true;
    } else {
        holding_button = false;
    }

    float speed_scale = MAX_SPEED / speed_divider;
    vw *= speed_scale;
    vx *= speed_scale;
    vy *= speed_scale;

    update_speed();
}
