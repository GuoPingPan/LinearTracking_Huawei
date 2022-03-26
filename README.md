# LinearTracking_Huawei
The Huawei Auto car partial jobs.

这个项目记录的是2021年华为云人工智能挑战赛——无人车挑战赛中道路检测、避障、倒车的代码

## Structure
- calibration_bev_fitcurve-1: 记录的是 `相机的标定`、`转化为鸟瞰图`、用 `飞思卡尔的曲线拟合` 思想的代码
- laserTracking: 利用 `激光雷达完成道路检测` 的代码，本来是以为赛道有墙的，但后面只能用在过桥，但实际上也没有使用
- lineTrack-2: 是 `相机道路线检测的开发代码` 版本
- lintracking:  `最终的车道线检测` 代码和 `倒车模块` 代码
