#ifndef PPM_H
#define PPM_H

#include "Arduino.h"
#include "utils.h"

#define PPM_PIN 40
#define NUM_CHANNELS 8 // 输出通道
#define SYNC_GAP 3000

#define PPM_RIGHT_STICK_LR filteredPPMValues[0] // 右摇杆左右 (Left≈1000, Right≈2000)
#define PPM_RIGHT_STICK_UD filteredPPMValues[1] // 右摇杆上下 (Down≈1000, Up≈2000)
#define PPM_LEFT_STICK_UD filteredPPMValues[2]  // 左摇杆上下 (Down≈1000, Up≈2000)
#define PPM_LEFT_STICK_LR filteredPPMValues[3]  // 左摇杆左右 (Left≈1000, Right≈2000)
#define PPM_LEFT_KNOB filteredPPMValues[4]      // 左旋钮 (Left≈1000, Right≈2000)
#define PPM_RIGHT_KNOB filteredPPMValues[5]     // 右旋钮 (Left≈2000, Right≈1000) — 注意方向相反
#define PPM_SWITCH_LEFT filteredPPMValues[6]    // 左拨杆 (Down≈19xx, Up≈10xx)
#define PPM_SWITCH_RIGHT filteredPPMValues[7]   // 右拨杆 (Down≈19xx, Middle≈14xx, Up≈10xx)

enum class PPM_SWITCH_POS : uint8_t
{
    UP = 0,
    MIDDLE,
    DOWN
};

enum class PPM_KNOB_MODE : uint8_t
{
    ShakeShoulder = 0,
    Jump,
    Normal
};

enum class JUMPING_PROCESS : uint8_t
{
    READY = 0,
    JUMPING,
    LAND
};

extern uint16_t ppmValues[];
extern int filteredPPMValues[];
extern uint8_t currentChannel;
extern uint32_t lastTime;

extern uint8_t enableHubMotor;     // 轮毂电机使能标志位
extern PPM_KNOB_MODE knobMode;     // 左旋钮模式控制
extern JUMPING_PROCESS jumpProcess; // 跳跃过程

void ppm_init();
void storeFilteredPPMData();
void IRAM_ATTR onPPMInterrupt();

PPM_SWITCH_POS getSwitchPos(int switchValue);
void remoteSwitch();

#endif