#include "mapping/coordinate_converter.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <algorithm>

// 构造函数，新增高度参数（meters），默认值0表示地面高度
CoordinateConverter::CoordinateConverter(double origin_latitude, double origin_longitude, double height)
    : origin_latitude_(origin_latitude),
      origin_longitude_(origin_longitude), 
      height_(height),
      proj_(origin_latitude, origin_longitude, height)// 使用GeographicLib的三参数构造
{
}

// 动态更新坐标系参数，新增高度参数（meters）
void CoordinateConverter::updateCoordinate(double new_origin_latitude, double new_origin_longitude, double new_height)
{
    origin_latitude_ = new_origin_latitude;
    origin_longitude_ = new_origin_longitude;
    height_ = new_height;
    proj_ = GeographicLib::LocalCartesian(origin_latitude_, origin_longitude_, height_); // 重建投影
}

// 将经纬度转换为局部笛卡尔坐标
void CoordinateConverter::LatLon2Map(double latitude, double longitude, double& x, double& y) const
{
    double z = height_;
    proj_.Forward(latitude, longitude, height_, x, y, z);
}









// 将局部笛卡尔坐标转换为经纬度
void CoordinateConverter::Map2LatLon(double x, double y, double& latitude, double& longitude) const
{
    double height = height_;
    proj_.Reverse(x, y, height_, latitude, longitude, height);
}

// 添加一对对应点（Marvelmind坐标，ENU坐标）
void CoordinateCalibration::addCalibrationPoint(double marvel_x, double marvel_y, double enu_x, double enu_y) {
    marvel_points_.push_back(std::make_pair(marvel_x, marvel_y));
    enu_points_.push_back(std::make_pair(enu_x, enu_y));
}
// 计算转换参数 - 改进版本
bool CoordinateCalibration::computeTransformation() {
    if (marvel_points_.size() < 3 || enu_points_.size() < 3) {
        calibrated_ = false;
        return false;
    }
    
    size_t n = marvel_points_.size();
    
    // 先计算所有点对之间的距离，用于检测异常值
    std::vector<std::pair<int, int>> point_pairs;
    std::vector<double> marvel_distances, enu_distances;
    std::vector<double> distance_ratios;
    
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double marvel_dx = marvel_points_[i].first - marvel_points_[j].first;
            double marvel_dy = marvel_points_[i].second - marvel_points_[j].second;
            double marvel_dist = sqrt(marvel_dx * marvel_dx + marvel_dy * marvel_dy);
            
            double enu_dx = enu_points_[i].first - enu_points_[j].first;
            double enu_dy = enu_points_[i].second - enu_points_[j].second;
            double enu_dist = sqrt(enu_dx * enu_dx + enu_dy * enu_dy);
            
            if (marvel_dist > 1e-6 && enu_dist > 1e-6) {
                point_pairs.push_back(std::make_pair(i, j));
                marvel_distances.push_back(marvel_dist);
                enu_distances.push_back(enu_dist);
                distance_ratios.push_back(enu_dist / marvel_dist);
            }
        }
    }
    
    // 计算距离比率的中位数作为初始缩放因子
    if (!distance_ratios.empty()) {
        std::vector<double> sorted_ratios = distance_ratios;
        std::sort(sorted_ratios.begin(), sorted_ratios.end());
        double median_scale = sorted_ratios[sorted_ratios.size() / 2];
        
        // 使用更严格的阈值移除异常值（距离比率偏离中位数太远的点对）
        double threshold = median_scale * 0.3; // 更严格的阈值
        std::vector<bool> valid_pairs(distance_ratios.size(), true);
        int valid_count = 0;
        
        for (size_t i = 0; i < distance_ratios.size(); ++i) {
            if (fabs(distance_ratios[i] - median_scale) > threshold) {
                valid_pairs[i] = false;
            } else {
                valid_count++;
            }
        }
        
        // 如果有效点对数量足够，则使用这些点对进行标定
        if (valid_count >= 3) {
            // 重新构建点集，只包含有效点对中的点
            std::vector<bool> valid_points(n, false);
            for (size_t i = 0; i < point_pairs.size(); ++i) {
                if (valid_pairs[i]) {
                    valid_points[point_pairs[i].first] = true;
                    valid_points[point_pairs[i].second] = true;
                }
            }
            
            std::vector<std::pair<double, double>> filtered_marvel_points;
            std::vector<std::pair<double, double>> filtered_enu_points;
            
            for (size_t i = 0; i < n; ++i) {
                if (valid_points[i]) {
                    filtered_marvel_points.push_back(marvel_points_[i]);
                    filtered_enu_points.push_back(enu_points_[i]);
                }
            }
            
            // 如果过滤后的点足够多，则使用过滤后的点
            if (filtered_marvel_points.size() >= 3) {
                marvel_points_ = filtered_marvel_points;
                enu_points_ = filtered_enu_points;
                n = marvel_points_.size();
            }
        }
    }
    
    // 使用加权最小二乘法进行拟合
    // 构建方程 Ax = b，其中x包含[scale*cos(theta), scale*sin(theta), tx, ty]
    Eigen::MatrixXd A(2 * n, 4);
    Eigen::VectorXd b(2 * n);
    Eigen::MatrixXd W(2 * n, 2 * n);  // 权重矩阵
    W.setZero();
    
    // 计算每对点的权重（基于到中心点的距离）
    double marvel_center_x = 0, marvel_center_y = 0;
    double enu_center_x = 0, enu_center_y = 0;
    
    for (size_t i = 0; i < n; ++i) {
        marvel_center_x += marvel_points_[i].first;
        marvel_center_y += marvel_points_[i].second;
        enu_center_x += enu_points_[i].first;
        enu_center_y += enu_points_[i].second;
    }
    
    marvel_center_x /= n;
    marvel_center_y /= n;
    enu_center_x /= n;
    enu_center_y /= n;
    
    // 填充矩阵A、向量b和权重矩阵W
    for (size_t i = 0; i < n; ++i) {
        // 计算权重（距离中心点越远权重越高）
        double marvel_dist_sq = (marvel_points_[i].first - marvel_center_x) * (marvel_points_[i].first - marvel_center_x) +
                               (marvel_points_[i].second - marvel_center_y) * (marvel_points_[i].second - marvel_center_y);
        double weight = 1.0 + marvel_dist_sq;  // 距离越远权重越高
        
        // 对于x坐标: marvel_x * s*cos(theta) - marvel_y * s*sin(theta) + tx = enu_x
        A(2 * i, 0) = marvel_points_[i].first;      // marvel_x
        A(2 * i, 1) = -marvel_points_[i].second;    // -marvel_y
        A(2 * i, 2) = 1.0;                          // tx
        A(2 * i, 3) = 0.0;                          // 无ty项
        
        // 对于y坐标: marvel_y * s*cos(theta) + marvel_x * s*sin(theta) + ty = enu_y
        A(2 * i + 1, 0) = marvel_points_[i].second; // marvel_y
        A(2 * i + 1, 1) = marvel_points_[i].first;  // marvel_x
        A(2 * i + 1, 2) = 0.0;                      // 无tx项
        A(2 * i + 1, 3) = 1.0;                      // ty
        
        b(2 * i) = enu_points_[i].first;            // enu_x
        b(2 * i + 1) = enu_points_[i].second;       // enu_y
        
        W(2 * i, 2 * i) = weight;
        W(2 * i + 1, 2 * i + 1) = weight;
    }
    
    // 使用加权最小二乘法求解: (A^T * W * A) * x = A^T * W * b
    Eigen::MatrixXd AtW = A.transpose() * W;
    Eigen::MatrixXd AtWA = AtW * A;
    Eigen::VectorXd AtWb = AtW * b;
    
    // 求解线性方程组
    Eigen::VectorXd solution = AtWA.ldlt().solve(AtWb);
    
    // 提取变换参数
    double s_cos = solution(0);  // scale * cos(theta)
    double s_sin = solution(1);  // scale * sin(theta)
    offset_x_ = solution(2);     // tx
    offset_y_ = solution(3);     // ty
    
    // 计算缩放因子和旋转角度
    scale_ = sqrt(s_cos * s_cos + s_sin * s_sin);
    if (scale_ > 1e-10) {
        rotation_ = atan2(s_sin, s_cos);
    } else {
        scale_ = 1.0;
        rotation_ = 0.0;
    }
    
    calibrated_ = true;
    return true;
}



// 将Marvelmind坐标转换为ENU坐标
void CoordinateCalibration::marvelToENU(double marvel_x, double marvel_y, double& enu_x, double& enu_y) const {
    if (!calibrated_) {
        // 如果没有标定，直接返回原始坐标
        enu_x = marvel_x;
        enu_y = marvel_y;
        return;
    }
    
    // 应用缩放、旋转和平移变换
    double rotated_x = marvel_x * cos(rotation_) - marvel_y * sin(rotation_);
    double rotated_y = marvel_x * sin(rotation_) + marvel_y * cos(rotation_);
    
    enu_x = scale_ * rotated_x + offset_x_;
    enu_y = scale_ * rotated_y + offset_y_;
}

// 新增main函数入口点，用于编译测试

// int main(int argc, char** argv)
// {
//     // 示例：创建CoordinateConverter对象并进行转换
//     CoordinateConverter converter(23.5, 100.0, 10.0);
    
//     double x, y;
//     converter.LonLat2Map(23.5, 100.2, x, y);
//     std::cout << "Converted coordinates: (" << x << ", " << y << ")" << std::endl;

//     double longitude, latitude;
//     converter.Map2LonLat(x, y, latitude, longitude);
//     std::cout << "Reversed coordinates: (" << latitude << ", " << longitude << ")" << std::endl;

//     return 0; 
// }