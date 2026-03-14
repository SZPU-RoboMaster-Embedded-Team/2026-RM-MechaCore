# FreeRTOS任务优先级调整指南
## 2026Engineer_SelfControl_BoardA_FreeRTOS项目

## 一、FreeRTOS优先级基础

### 优先级范围
- FreeRTOS优先级：0-31（数字越大，优先级越高）
- CMSIS-RTOS2优先级枚举：
  ```c
  osPriorityIdle          = 1,   // 空闲任务
  osPriorityLow           = 8,   // 低优先级
  osPriorityBelowNormal   = 16,  // 低于正常
  osPriorityNormal        = 24,  // 正常优先级
  osPriorityAboveNormal   = 32,  // 高于正常
  osPriorityHigh          = 40,  // 高优先级
  osPriorityRealtime      = 48,  // 实时优先级
  ```

## 二、项目任务分析

根据你的项目（工程师机器人控制系统），推测可能包含以下任务：

### 1. **ArmTask（机械臂控制任务）**
- **功能**：机械臂关节控制、PID计算、力矩控制
- **实时性要求**：高（需要快速响应，保证控制精度）
- **推荐优先级**：`osPriorityHigh (40)` 或 `osPriorityAboveNormal (32)`
- **控制周期**：1-5ms

### 2. **RemoteTask（遥控任务）**
- **功能**：接收遥控器指令、解析DT7数据
- **实时性要求**：高（安全相关，需要及时响应）
- **推荐优先级**：`osPriorityHigh (40)` 或 `osPriorityRealtime (48)`
- **控制周期**：10-20ms

### 3. **CommunicationTask（通信任务）**
- **功能**：板间通信、数据收发
- **实时性要求**：中等
- **推荐优先级**：`osPriorityAboveNormal (32)` 或 `osPriorityNormal (24)`
- **控制周期**：20-50ms

### 4. **HostSerialTask（上位机串口任务）**
- **功能**：与上位机通信、调试数据发送
- **实时性要求**：低
- **推荐优先级**：`osPriorityNormal (24)` 或 `osPriorityBelowNormal (16)`
- **控制周期**：50-100ms

### 5. **其他可能的任务**
- **LED指示任务**：`osPriorityLow (8)`
- **监控任务**：`osPriorityBelowNormal (16)`
- **日志任务**：`osPriorityLow (8)`

## 三、推荐的优先级配置方案

### 方案A：标准配置（推荐）
```c
// 在freertos.c或任务创建文件中

// 1. 遥控任务（最高优先级 - 安全相关）
#define REMOTE_TASK_PRIORITY        osPriorityRealtime      // 48

// 2. 机械臂控制任务（高优先级 - 实时控制）
#define ARM_TASK_PRIORITY           osPriorityHigh          // 40

// 3. CAN通信任务（高优先级 - 电机通信）
#define CAN_TASK_PRIORITY           osPriorityHigh          // 40

// 4. 板间通信任务（中等优先级）
#define COMMUNICATION_TASK_PRIORITY osPriorityAboveNormal   // 32

// 5. 上位机串口任务（低优先级）
#define HOST_SERIAL_TASK_PRIORITY   osPriorityNormal        // 24

// 6. LED指示任务（最低优先级）
#define LED_TASK_PRIORITY           osPriorityLow           // 8
```

### 方案B：高性能配置（对实时性要求极高）
```c
#define REMOTE_TASK_PRIORITY        osPriorityRealtime      // 48
#define ARM_TASK_PRIORITY           osPriorityRealtime      // 48
#define CAN_TASK_PRIORITY           osPriorityHigh          // 40
#define COMMUNICATION_TASK_PRIORITY osPriorityAboveNormal   // 32
#define HOST_SERIAL_TASK_PRIORITY   osPriorityBelowNormal   // 16
#define LED_TASK_PRIORITY           osPriorityLow           // 8
```

### 方案C：平衡配置（CPU负载较高时）
```c
#define REMOTE_TASK_PRIORITY        osPriorityHigh          // 40
#define ARM_TASK_PRIORITY           osPriorityAboveNormal   // 32
#define CAN_TASK_PRIORITY           osPriorityAboveNormal   // 32
#define COMMUNICATION_TASK_PRIORITY osPriorityNormal        // 24
#define HOST_SERIAL_TASK_PRIORITY   osPriorityBelowNormal   // 16
#define LED_TASK_PRIORITY           osPriorityLow           // 8
```

## 四、任务创建示例代码

### 在freertos.c中修改任务创建

```c
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

// 任务句柄
osThreadId_t ArmTaskHandle;
osThreadId_t RemoteTaskHandle;
osThreadId_t CommunicationTaskHandle;
osThreadId_t HostSerialTaskHandle;

// 任务属性定义
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  .stack_size = 512 * 4,  // 2KB栈空间
  .priority = (osPriority_t) osPriorityHigh,  // 高优先级
};

const osThreadAttr_t RemoteTask_attributes = {
  .name = "RemoteTask",
  .stack_size = 256 * 4,  // 1KB栈空间
  .priority = (osPriority_t) osPriorityRealtime,  // 实时优先级
};

const osThreadAttr_t CommunicationTask_attributes = {
  .name = "CommunicationTask",
  .stack_size = 512 * 4,  // 2KB栈空间
  .priority = (osPriority_t) osPriorityAboveNormal,  // 中高优先级
};

const osThreadAttr_t HostSerialTask_attributes = {
  .name = "HostSerialTask",
  .stack_size = 256 * 4,  // 1KB栈空间
  .priority = (osPriority_t) osPriorityNormal,  // 正常优先级
};

// 在MX_FREERTOS_Init()函数中创建任务
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ArmTask */
  ArmTaskHandle = osThreadNew(ArmTask, NULL, &ArmTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(RemoteTask, NULL, &RemoteTask_attributes);

  /* creation of CommunicationTask */
  CommunicationTaskHandle = osThreadNew(CommunicationTask, NULL, &CommunicationTask_attributes);

  /* creation of HostSerialTask */
  HostSerialTaskHandle = osThreadNew(HostSerialTask, NULL, &HostSerialTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}
```

## 五、动态调整优先级

### 运行时修改任务优先级

```c
// 提高ArmTask优先级
osThreadSetPriority(ArmTaskHandle, osPriorityRealtime);

// 降低HostSerialTask优先级
osThreadSetPriority(HostSerialTaskHandle, osPriorityLow);

// 获取当前任务优先级
osPriority_t current_priority = osThreadGetPriority(ArmTaskHandle);
```

### 临时提升优先级示例

```c
void ArmTask(void *argument)
{
    osPriority_t original_priority = osThreadGetPriority(NULL);
    
    for(;;)
    {
        // 正常运行
        osDelay(1);
        
        // 关键操作时临时提升优先级
        if (critical_operation_needed) {
            osThreadSetPriority(NULL, osPriorityRealtime);
            
            // 执行关键操作
            performCriticalOperation();
            
            // 恢复原优先级
            osThreadSetPriority(NULL, original_priority);
        }
    }
}
```

## 六、优先级调整原则

### 1. 基本原则
- **安全第一**：遥控、急停等安全相关任务优先级最高
- **实时控制**：电机控制、PID计算等需要高优先级
- **通信次之**：CAN、串口等通信任务中等优先级
- **显示最低**：LED、日志等非关键任务最低优先级

### 2. 避免优先级反转
```c
// 使用互斥量时启用优先级继承
osMutexAttr_t mutex_attr = {
  .attr_bits = osMutexPrioInherit,  // 优先级继承
};
osMutexId_t myMutex = osMutexNew(&mutex_attr);
```

### 3. 任务时间片配置
```c
// 在FreeRTOSConfig.h中配置
#define configUSE_PREEMPTION                1  // 使用抢占式调度
#define configUSE_TIME_SLICING              1  // 使用时间片轮转
#define configTICK_RATE_HZ                  1000  // 1ms时钟节拍
```

## 七、性能监控

### 1. 任务运行时间统计
```c
// 在FreeRTOSConfig.h中启用
#define configGENERATE_RUN_TIME_STATS       1
#define configUSE_TRACE_FACILITY            1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

// 获取任务统计信息
void printTaskStats(void) {
    char buffer[512];
    vTaskGetRunTimeStats(buffer);
    printf("%s\n", buffer);
}
```

### 2. 栈使用监控
```c
// 检查任务栈使用情况
UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(ArmTaskHandle);
printf("ArmTask stack remaining: %d bytes\n", stack_high_water_mark * 4);
```

## 八、调试建议

### 1. 优先级冲突检测
- 使用SEGGER SystemView或Tracealyzer工具
- 监控任务切换频率
- 检查是否有任务饥饿现象

### 2. 优化步骤
1. 从推荐配置开始
2. 使用性能监控工具观察
3. 根据实际情况微调
4. 测试极限情况（满负载）
5. 验证安全性和稳定性

## 九、常见问题

### Q1: 多个任务优先级相同会怎样？
A: 相同优先级的任务会按时间片轮转调度，每个任务运行一个时间片后切换。

### Q2: 优先级设置过高会有什么问题？
A: 可能导致低优先级任务饥饿，系统响应不均衡。

### Q3: 如何确定最佳优先级？
A: 通过实际测试，监控任务响应时间和CPU占用率，逐步调整。

## 十、项目特定建议

对于你的工程师机器人项目：

1. **ArmTask（机械臂控制）**
   - 优先级：`osPriorityHigh (40)`
   - 周期：1ms（1kHz控制频率）
   - 栈大小：2-4KB

2. **RemoteTask（遥控）**
   - 优先级：`osPriorityRealtime (48)`
   - 周期：10ms
   - 栈大小：1KB

3. **CommunicationTask（通信）**
   - 优先级：`osPriorityAboveNormal (32)`
   - 周期：20ms
   - 栈大小：2KB

4. **HostSerialTask（调试）**
   - 优先级：`osPriorityNormal (24)`
   - 周期：50ms
   - 栈大小：1KB

## 十一、配置文件位置

在你的项目中，任务配置通常在以下文件：
- `Core/Src/freertos.c` - 任务创建和配置
- `Core/Inc/FreeRTOSConfig.h` - FreeRTOS系统配置
- `User/Task/*.cpp` - 各任务实现

修改这些文件后重新编译即可生效。