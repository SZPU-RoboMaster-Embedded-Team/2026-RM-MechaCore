#ifndef __BUZZER_HPP
#define __BUZZER_HPP

#include "main.h"

#define MAX_PSC             1000
#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

// ========== 【配置区】在这里更改你想播放的歌曲！==========
// 
// 可选歌曲列表：
//   中文歌曲：
//     Song_MoChouXiang      - 莫愁乡
//     Song_YiXiaoJiangHu    - 一笑江湖
//     Song_Xixirangrang     - 嘻嘻嚷嚷
//     Song_You              - You
//     Song_SeeYouAgain      - See You Again (速度与激情7)
//   
//   游戏音乐：
//     Song_SuperMarioBros   - 超级马里奥
//     Song_Tetris           - 俄罗斯方块
//   
//   日文歌曲：
//     Song_YebenRacingIntoTheNight - 夜奔/天堂に駆ける (YOASOBI)
//   
//   电影音乐：
//     Song_PiratesOfTheCaribbean - 加勒比海盗 (推荐！)
//   
//   其他：
//     Song_CanonInD         - 卡农
//     Song_NokiaTune        - Nokia铃声
// 
#define ACTIVE_SONG Song_NokiaTune

#define P0 	0	// 休止符频率

// ========== 原始音高定义（已注释，适用于标准扬声器）==========
// #define LL1    131  
// #define LL1_S  139
// #define LL2    147
// #define LL2_S  156
// #define LL3    165
// #define LL4    175
// #define LL4_S  185
// #define LL5    196
// #define LL5_S  208
// #define LL6    220
// #define LL6_S  233
// #define LL7    247

// #define L1     262  
// #define L1_S   277
// #define L2     294
// #define L2_S   311
// #define L3     330
// #define L4     349
// #define L4_S   370
// #define L5     392
// #define L5_S   415
// #define L6     440
// #define L6_S   466
// #define L7     494

// #define M1     523  
// #define M1_S   554
// #define M2     587
// #define M2_S   622
// #define M3     659
// #define M4     698
// #define M4_S   740
// #define M5     784
// #define M5_S   831
// #define M6     880
// #define M6_S   932
// #define M7     988

// #define H1     1047 
// #define H1_S   1109
// #define H2     1175
// #define H2_S   1245
// #define H3     1319
// #define H4     1397
// #define H4_S   1480
// #define H5     1568
// #define H5_S   1661
// #define H6     1760
// #define H6_S   1865
// #define H7     1976

// #define HH1    2094 
// #define HH1_S  2217
// #define HH2    2350
// #define HH2_S  2489
// #define HH3    2638
// #define HH4    2794
// #define HH4_S  2960
// #define HH5    3136
// #define HH5_S  3322
// #define HH6    3520
// #define HH6_S  3729
// #define HH7    3952

// ========== 优化音高定义（针对2700Hz蜂鸣器，整体提升一个八度）==========
// 设计思路：将所有音符提升一个八度，使主要音域更接近2700Hz谐振频率
// 优化效果：
//   - 减少低音失真（Nokia铃声的LL音符问题）
//   - 提升整体音量和音质
//   - 主要音域落在1000-4000Hz，更接近2700Hz最佳响应区

// 倍低音区（LL）：262-494Hz（原L音区）
#define LL1    262  
#define LL1_S  277
#define LL2    294
#define LL2_S  311
#define LL3    330
#define LL4    349
#define LL4_S  370
#define LL5    392
#define LL5_S  415
#define LL6    440
#define LL6_S  466
#define LL7    494

// 低音区（L）：523-988Hz（原M音区）
#define L1     523  
#define L1_S   554
#define L2     587
#define L2_S   622
#define L3     659
#define L4     698
#define L4_S   740
#define L5     784
#define L5_S   831
#define L6     880
#define L6_S   932
#define L7     988

// 中音区（M）：1047-1976Hz（原H音区）
#define M1     1047  
#define M1_S   1109
#define M2     1175
#define M2_S   1245
#define M3     1319
#define M4     1397
#define M4_S   1480
#define M5     1568
#define M5_S   1661
#define M6     1760
#define M6_S   1865
#define M7     1976

// 高音区（H）：2094-3952Hz（原HH音区，最接近2700Hz！）
#define H1     2094 
#define H1_S   2217
#define H2     2350
#define H2_S   2489
#define H3     2638  // 非常接近2700Hz谐振点
#define H4     2794  // 非常接近2700Hz谐振点
#define H4_S   2960
#define H5     3136
#define H5_S   3322
#define H6     3520
#define H6_S   3729
#define H7     3952

// 倍高音区（HH）：4188-7904Hz（新增，提升一个八度）
#define HH1    4188 
#define HH1_S  4434
#define HH2    4700
#define HH2_S  4978
#define HH3    5276
#define HH4    5588
#define HH4_S  5920
#define HH5    6272
#define HH5_S  6644
#define HH6    7040
#define HH6_S  7458
#define HH7    7904

typedef struct
{
    uint16_t frequency;
    float period;
} Bate;

extern uint8_t playState;
extern bool isBuzzerPlaying;

void Buzzer_Timers_Init(void);
void Handle_Buzzer(void);

typedef struct {
    const Bate* notes;  // 音符数组指针
    uint32_t length;    // 数组长度
    uint16_t bpm;       // 歌曲BPM
} Song_t;

extern const Song_t* current_song;

extern const Bate MoChouXiang[];
extern const Bate YiXiaoJiangHu[];
extern const Bate Xixirangrang[];
extern const Bate You[];
extern const Bate SeeYouAgain[];
extern const Bate CanonInD[];
extern const Bate NokiaTune[];
extern const Bate SuperMarioBros[];
extern const Bate Tetris[];
extern const Bate YebenRacingIntoTheNight[];
extern const Bate PiratesOfTheCaribbean[];
extern const Bate VisionConnected[];
extern const Bate VisionDisconnected[];
extern const Bate VisionOfflineBeep[];
extern const Bate VisionTargetLocked[];

// 歌曲的结构体封装声明
extern const Song_t Song_MoChouXiang;
extern const Song_t Song_YiXiaoJiangHu;
extern const Song_t Song_Xixirangrang;
extern const Song_t Song_You;
extern const Song_t Song_SeeYouAgain;
extern const Song_t Song_CanonInD;
extern const Song_t Song_NokiaTune;
extern const Song_t Song_SuperMarioBros;
extern const Song_t Song_Tetris;
extern const Song_t Song_YebenRacingIntoTheNight;
extern const Song_t Song_PiratesOfTheCaribbean;
extern const Song_t Song_VisionConnected;
extern const Song_t Song_VisionDisconnected;
extern const Song_t Song_VisionOfflineBeep;
extern const Song_t Song_VisionTargetLocked;

#endif
