#ifndef COORDINATE_CONVERTER_H
#define COORDINATE_CONVERTER_H
#include <GeographicLib/LocalCartesian.hpp>
#include <vector>
#include <utility>
// 这是一个坐标转换类，用于将经纬度转换为局部笛卡尔坐标，并支持动态更新坐标系参数
// 支持高度参数（meters），默认值为0表示地面高度
// 使用GeographicLib库进行坐标转换
// x轴正方向为经度正方向，y轴正方向为纬度正方向，z轴为高度

class CoordinateConverter {
public:
    // 构造函数，提供原点的纬度、经度和高度参数（meters）
    CoordinateConverter(double origin_latitude = 0.0, double origin_longitude = 0.0, double height = 0.0);
    
    // 将经纬度转换为局部笛卡尔坐标
    void LatLon2Map(double latitude, double longitude, double& x, double& y) const;
    
    // 将局部笛卡尔坐标转换为经纬度
    void Map2LatLon(double x, double y, double& latitude, double& longitude) const;
    
    // 动态更新坐标系参数，新增高度参数（meters）
    void updateCoordinate(double new_origin_latitude, double new_origin_longitude, double new_height = 0.0);
    
private:
    double origin_latitude_ = 0.0;    // 原点纬度
    double origin_longitude_ = 0.0;   // 原点经度
    double height_ = 0.0;             // 地图平面高度
    GeographicLib::LocalCartesian proj_;
};

// 坐标标定类，用于将Marvelmind坐标系转换为ENU坐标系
class CoordinateCalibration {
public:
    // 添加一对对应点（Marvelmind坐标，ENU坐标）
    void addCalibrationPoint(double marvel_x, double marvel_y, double enu_x, double enu_y);
    
    // 计算转换参数
    bool computeTransformation();
    
    // 将Marvelmind坐标转换为ENU坐标
    void marvelToENU(double marvel_x, double marvel_y, double& enu_x, double& enu_y) const;
    
    // 获取转换参数
    double getScale() const { return scale_; }
    double getRotation() const { return rotation_; }
    double getOffsetX() const { return offset_x_; }
    double getOffsetY() const { return offset_y_; }
    
    // 检查是否有足够的点进行标定
    bool isCalibrated() const { return calibrated_; }

private:
    // 存储标定点对
    std::vector<std::pair<double, double>> marvel_points_;
    std::vector<std::pair<double, double>> enu_points_;
    
    // 转换参数
    double scale_ = 1.0;       // 缩放因子
    double rotation_ = 0.0;    // 旋转角度（弧度）
    double offset_x_ = 0.0;    // X轴偏移
    double offset_y_ = 0.0;    // Y轴偏移
    bool calibrated_ = false;   // 是否已完成标定
};



#endif // COORDINATE_CONVERTER_H