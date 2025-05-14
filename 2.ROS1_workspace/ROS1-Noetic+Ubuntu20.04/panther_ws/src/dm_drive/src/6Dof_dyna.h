#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;
using namespace std;

// 常量定义
const double PI = M_PI;
const Vector3d GRAVITY(0, 0, 9.8); // 重力向量
const Vector3d Z_AXIS(0, 0, 1);    // Z轴单位向量

// 质量
// const double MASS[6] = {0.638856, 0.841, 0.7295, 0.5665, 0.64, 0.000000};//无末端
const double MASS[6] = {0.638856, 0.841, 0.73, 0.46, 0.39, 0.450};//夹爪末端

// 改进型D-H参数 (常量预先计算)
// a_{i-1},        alpha_{i-1},     d_i,         theta_i
const double DH[6][4] = {
    {0, 0, 0.1088, 0},
    {0, -PI / 2, 0, 0},
    {0.180, PI, 0, 0},
    {0.18881, PI, 0, 0},
    {0.076, PI / 2, -0.011, 0},
    {0, PI / 2, 0.110, 0}};

// 各关节p
const Vector3d p[7] = {
    Vector3d(0.0, 0.0, 0.1005),    // p10
    Vector3d(0.0, 0.0, 0.0),       // p12
    Vector3d(0.1800, 0.0, 0.0),    // p32
    Vector3d(0.1888, 0.0, 0.0),    // p43
    Vector3d(0.080, 0.0110, 0.0), // p54
    Vector3d(0.0, -0.0400, 0.0),   // p65
    Vector3d(0.0, 0.0, 0.0)        // p76
};

// 质心位置 (相对于连杆坐标系)
const Vector3d pc[6] = {
    Vector3d(0, -0.008491, 0.034259), // 连杆1
    Vector3d(0.124560, 0.000000, 0.017699),
    Vector3d(0.15, 0.007521, 0.017264),
    Vector3d(0.08, -0.053910, 0.004474),
    Vector3d(0.013, 0.000000,  -0.007913),
    Vector3d(0.000018, -0.000159, 0.09)
};

// 惯性矩阵(动力学用到)
const Matrix3d INERTIA[6] = {
    (Matrix3d() << 0.000531, 0, 0, 0, 0.000473, 0, 0, 0, 0.000353).finished(),
    (Matrix3d() << 0.000469, 0.000000, -0.000555, 0.000000, 0.004434, 0.000000, -0.000555, 0.000000, 0.004401).finished(),
    (Matrix3d() << 0.000544, -0.000276, -0.000546, -0.000276, 0.004276, 0.000077, -0.000546, 0.000077, 0.004342).finished(),
    (Matrix3d() << 0.000559, -0.000294,  -0.000102, -0.000294, 0.000489, 0.000104, -0.000102, 0.000104, 0.000773).finished(),
    (Matrix3d() << 0.000247, 0.0, 0.000015, 0.0, 0.000250, 0.000000, 0.000015, 0.000000, 0.000171).finished(),
    (Matrix3d() << 0.000395, 0.000002, 0.0, 0.000002, 0.000760, 0.000002, 0.0, 0.000002, 0.000444).finished()};


// 计算重力补偿力矩 (静态情况)(只用到重心和质量)
VectorXd compute_gravity_compensation(const double q[6])
{
    // 初始化运动学链
    Matrix3d R01, R12, R23, R34, R45, R56; // R_i^{i-1}
    Matrix3d R10, R21, R32, R43, R54, R65; // R_i^{i-1}
    Vector3d w11, w22, w33, w44, w55, w66;
    Vector3d w11d, w22d, w33d, w44d, w55d, w66d;
    Vector3d v11d, v22d, v33d, v44d, v55d, v66d;
    Vector3d vc11d, vc22d, vc33d, vc44d, vc55d, vc66d;
    Vector3d F11, F22, F33, F44, F55, F66;
    Vector3d f11, f22, f33, f44, f55, f66;
    Vector3d n11, n22, n33, n44, n55, n66;

    //初始theta偏置
    double q_offset[6];
    q_offset[0] = q[0];
    q_offset[1] = q[1] + PI;
    q_offset[2] = q[2] - 162.43*PI/180.0;
    q_offset[3] = q[3] + 17.57*PI/180.0;
    q_offset[4] = q[4] + PI/2;
    q_offset[5] = q[5];

    R01 << cos(q_offset[0]), -sin(q_offset[0]), 0.0,
        sin(q_offset[0]), cos(q_offset[0]), 0.0,
        0.0, 0.0, 1;

    R12 << cos(q_offset[1]), -sin(q_offset[1]), 0.0,
        0.0, 0.0, 1,
        -sin(q_offset[1]), -cos(q_offset[1]), 0.0;

    R23 << cos(q_offset[2]), -sin(q_offset[2]), 0.0,
        -sin(q_offset[2]), -cos(q_offset[2]), 0.0,
        0.0, 0.0, -1;

    R34 << cos(q_offset[3]), -sin(q_offset[3]), 0.0,
        -sin(q_offset[3]), -cos(q_offset[3]), 0.0,
        0.0, 0.0, -1;

    R45 << cos(q_offset[4]), -sin(q_offset[4]), 0.0,
        0.0, 0.0, 1,
        -sin(q_offset[4]), -cos(q_offset[4]), 0.0;

    R56 << cos(q_offset[5]), -sin(q_offset[5]), 0.0,
        0.0, 0.0, -1,
        sin(q_offset[5]), cos(q_offset[5]), 0.0;

    R10 = R01.transpose();
    R21 = R12.transpose();
    R32 = R23.transpose();
    R43 = R34.transpose();
    R54 = R45.transpose();
    R65 = R56.transpose();

    // 外推初始化 (基座)
    Vector3d w00 = Vector3d::Zero();  // 基座角速度
    Vector3d v00 = Vector3d::Zero();  // 基座线速度
    Vector3d w00d = Vector3d::Zero(); // 基座角加速度
    Vector3d v00d = GRAVITY;          // 基座线加速度 (包含重力)

    w11 = Vector3d::Zero();
    w22 = Vector3d::Zero();
    w33 = Vector3d::Zero();
    w44 = Vector3d::Zero();
    w55 = Vector3d::Zero();
    w66 = Vector3d::Zero();

    w11d = Vector3d::Zero();
    w22d = Vector3d::Zero();
    w33d = Vector3d::Zero();
    w44d = Vector3d::Zero();
    w55d = Vector3d::Zero();
    w66d = Vector3d::Zero();

    v11d = R10 * v00d;
    v22d = R21 * v11d;
    v33d = R32 * v22d;
    v44d = R43 * v33d;
    v55d = R54 * v44d;
    v66d = R65 * v55d;

    vc11d = v11d;
    vc22d = v22d;   
    vc33d = v33d;
    vc44d = v44d;
    vc55d = v55d;
    vc66d = v66d;

    F11 = MASS[0] * vc11d;
    F22 = MASS[1] * vc22d;
    F33 = MASS[2] * vc33d;
    F44 = MASS[3] * vc44d;
    F55 = MASS[4] * vc55d;
    F66 = MASS[5] * vc66d;

    Vector3d f77 = Vector3d::Zero(); // 末端外力
    Vector3d n77 = Vector3d::Zero(); // 末端外力矩
    VectorXd tau(6);

    f66 = F66;
    f55 = R56 * f66 + F55;
    f44 = R45 * f55 + F44;
    f33 = R34 * f44 + F33;
    f22 = R23 * f33 + F22;
    f11 = R12 * f22 + F11;


    n66 = pc[6 - 1].cross(F66);
    n55 = (R56 * n66) + pc[5 - 1].cross(F55) + p[6 - 1].cross(R56 * f66);
    n44 = (R45 * n55) + pc[4 - 1].cross(F44) + p[5 - 1].cross(R45 * f55);
    n33 = (R34 * n44) + pc[3 - 1].cross(F33) + p[4 - 1].cross(R34 * f44);
    n22 = (R23 * n33) + pc[2 - 1].cross(F22) + p[3 - 1].cross(R23 * f33);
    n11 = (R12 * n22) + pc[1 - 1].cross(F11) + p[2 - 1].cross(R12 * f22);

    tau[5] = n66.dot(Z_AXIS);
    tau[4] = n55.dot(Z_AXIS);
    tau[3] = n44.dot(Z_AXIS);
    tau[2] = n33.dot(Z_AXIS);
    tau[1] = n22.dot(Z_AXIS);
    tau[0] = n11.dot(Z_AXIS);

    return tau;
}