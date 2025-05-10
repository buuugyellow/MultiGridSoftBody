# MultiGridSoftBody
## 架构
Render 项目为渲染模块，编译成 dll 供 MultiGridSoftBody 项目调用
注意 Render 的 dll 依赖于另外两个 dll 

## 测试模型

### 第一版

1. 3dmax 中创建长方体，长 20 宽 2 高 2，分段自定义，默认 YZ 翻转，因此长在 z 轴，x[-1,1] y[-1,1] z[-10,10]，导出三角形 .obj

2. meshlab 验证网格，转 .obj 为 .stl
3. gmsh 填充四面体，导出 .msh

相机 cam1 中 eye[20,0,0] target[0,0,0] up[0,1,0]

### 第二版

为了对齐坐标系，长方体 x[-1,1] y[-10,10] z[-1,1]，同时固定点也从原来的 [0,0,10] 改为 [0,10,0]

相机 cam2 中 eye[0,0,20] target[0,0,0] up[0,1,0]

模型以 Y_ 为前缀

为了控制变量测试碰撞，取消重力的作用
