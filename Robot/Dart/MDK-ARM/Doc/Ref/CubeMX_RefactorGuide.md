# CubeMX 重构配置指南

如果你打算采用 **MechaCore** 框架或对当前项目进行结构性重构，建议在 STM32CubeMX 中进行以下调整：

## 1. FreeRTOS (Middleware)
- **任务管理 (Tasks and Queues)**:
    - **现状**: 任务通过 CubeMX 生成为 `__weak` 函数并在外部实现。这虽然方便，但不利于 C++ 对象的生命周期管理。
    - **建议**: 减少在 CubeMX 中创建的任务数量。可以只保留一个 `DefaultTask` 作为入口，然后在 C++ 代码中使用 `osThreadNew` 或 `new` 来动态创建并初始化各个功能类实例。
- **内存配置**:
    - **Heap Size**: 当前为 `10240` (10KB)。C++ 框架（尤其是使用 `std::function` 或大量类对象时）对堆内存有一定要求。
    - **修改**: 建议将 `configTOTAL_HEAP_SIZE` 调大至 `32768` (32KB) 或更高（F427II 有 256KB RAM，空间充裕）。
- **优先级优化**:
    - 确保 **电机控制任务 (PIDControl)** 拥有足够高的优先级且不被阻塞。

## 2. 串口通信 (DMA & Interrupts)
- **DMA 模式切换 (重要)**:
    - **现状**: `USART1_RX` (遥控器) 使用的是 `DMA_NORMAL` 模式。
    - **修改**: 建议将所有接收通道 (RX) 改为 **`DMA_Circular`** 模式。
    - **理由**: `MechaCore` 框架通常配合 **串口空闲中断 (IDLE)** 使用。循环模式可以确保在高速数据流下不丢失字节，且无需在中断里频繁重启 DMA。
- **中断优先级**:
    - 确保 CAN 和 UART 的中断优先级数字 **大于等于 5** (即优先级低于 FreeRTOS 的系统调用阈值)。当前配置已经是 5，保持即可。

## 3. CAN 外设
- **过滤器配置**:
    - **CAN2 启动**: 如果未来需要增加 CAN2 接口，确保在 CAN1 的设置中配置好 `Slave Start Filter Bank`（通常设为 14），否则 CAN2 可能无法接收数据。

## 4. 代码生成设置 (Project Manager)
- **C++ 混合编译**:
    - 确保在工具链设置中允许 C++。由于你目前已经在运行 C++ 代码，这一点应该是 OK 的。
- **文件生成**:
    - 保持 `Generate peripheral initialization as a pair of '.c/.h' files per peripheral` 勾选，这对解耦底层驱动非常有帮助。

## 5. 总结维护建议
重构的核心目标是**“让 CubeMX 只管硬件初始化，逻辑交给 C++”**。
- 尽量不在 CubeMX 里创建复杂的 Queue 或 Semaphore，而是在 C++ 的单例或管理类中创建。
- 业务代码尽量写在 `APP_Task` 文件夹下，不要在 `Core/Src` 里的生成文件夹内逗留。
