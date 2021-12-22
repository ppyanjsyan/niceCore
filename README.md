# niceCore
The goal is to implement an out-of-order superscalar processor called "niceCore".


架构特性

指令集              RV32I
流水线发射宽度       4条指令/时钟周期
保留站              集中控制、共享结构，64项
寄存器重命名         物理寄存器与重命名寄存器共享结构，128项
物理寄存器堆         128个物理寄存器
重排序缓存           128项
执行单元             ALU、分支指令执行单元和load/store指令执行单元
指令预测             支持2级预测



