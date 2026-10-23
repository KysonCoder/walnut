/*
已通过“ce-se3_twist_ops.cpp”测试

se3_twist 命名空间
├── 数据类型定义
│   ├── scalar: double (基本数值类型)
│   ├── Vec3: 三维向量 [x,y,z]
│   ├── Vec6: 六维向量 [ωx,ωy,ωz,vx,vy,vz]
│   ├── Mat3: 3x3 矩阵
│   ├── Mat4: 4x4 矩阵
│   └── Mat6: 6x6 矩阵
│
├── Twist 类 (核心数据结构)
│   ├── data: 存储6个数值的数组
│   ├── omega(): 访问旋转部分 [ωx,ωy,ωz]
│   └── vel(): 访问移动部分 [vx,vy,vz]
│
├── 工具函数 (utils命名空间)
│   ├── 向量运算: cross, scale
│   ├── 矩阵运算: mat3_mul, mat3_scale, mat3_add
│   ├── 特殊转换: hat, vee
│   └── 单位矩阵: mat3_identity, mat4_identity
│
└── 核心运算类
    ├── Zero: 创建零旋量
    ├── Add: 旋量加法
    ├── Scale: 旋量标量乘法
    ├── LieBracket: 李括号运算
    ├── Exp: 指数映射 (运动 → 位置)
    ├── Log: 对数映射 (位置 → 运动)
    └── Adjoint: 伴随映射 (坐标系转换)

*/






#ifndef SE3_TWIST_OPS_HPP
#define SE3_TWIST_OPS_HPP

#include <array>
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace se3_twist {


struct CARTESIAN 
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // 重载 [] 运算符
    double& operator[](int index) {
        switch(index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("CARTESIAN index out of range");
        }
    }

    
};


struct QUATERNION 
{
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
};

typedef struct
{
    CARTESIAN pos;
    QUATERNION orient;
}POSE;


struct SOC_JOINTS
{
    double joint[7];

     // 重载 [] 运算符，直接访问 joint 数组
    double& operator[](int index) {
        return joint[index];
    }

    size_t size() {
        return 7;
    }
    
};

/**
 * @brief 定义基础数据类型
 * 
 * 使用 double 作为标量类型，定义三维向量 Vec3、六维向量 Vec6、
 * 3x3 矩阵 Mat3、4x4 矩阵 Mat4 和 6x6 矩阵 Mat6。
 */
using scalar = double;
using Vec3   = std::array<scalar, 3>;
using Vec4   = std::array<scalar, 4>;
using Vec5   = std::array<scalar, 5>;
using Vec6   = std::array<scalar, 6>;
using Vec8   = std::array<scalar, 8>;
using Vec10   = std::array<scalar, 10>;
using Mat3   = std::array<std::array<scalar, 3>, 3>;
using Mat4   = std::array<std::array<scalar, 4>, 4>;
using Mat5   = std::array<std::array<scalar, 5>, 5>;
using Mat6   = std::array<std::array<scalar, 6>, 6>;
using Mat8_5   = std::array<std::array<scalar, 5>, 8>;//八行五列

/**
 * @brief 旋量（Twist）结构体，表示 se(3) 李代数元素
 * 
 * 旋量 ξ ∈ se(3) 是一个 6 维向量，前三个分量表示角速度 ω ∈ ℝ³，
 * 后三个分量表示线速度 v ∈ ℝ³，即 ξ = [ω; v]。
 * 
 * 该结构体提供对 ω 和 v 的直接访问方法。
 */
struct Twist {
    Vec6 data{};  ///< 存储 6 维向量的数据

    /// 默认构造函数，初始化为零旋量
    Twist() = default;
    
    /// 从 Vec6 构造旋量
    explicit Twist(const Vec6& xi) : data(xi) {}
    
    /// 访问角速度分量 ω
    Vec3&       omega()       { return *reinterpret_cast<Vec3*>(&data[0]); }
    const Vec3& omega() const { return *reinterpret_cast<const Vec3*>(&data[0]); }
    
    /// 访问线速度分量 v
    Vec3&       vel()         { return *reinterpret_cast<Vec3*>(&data[3]); }
    const Vec3& vel()   const { return *reinterpret_cast<const Vec3*>(&data[3]); }
};

// ==================== 工具函数 ====================
namespace utils {

/**
 * @brief 创建 3x3 零矩阵
 * @return 所有元素为零的 3x3 矩阵
 */
inline Mat3 mat3_zero() {
    return {{{0,0,0}, {0,0,0}, {0,0,0}}};
}

/**
 * @brief 创建 4x4 零矩阵
 * @return 所有元素为零的 4x4 矩阵
 */
inline Mat4 mat4_zero() {
    return {{{0,0,0,0}, {0,0,0,0}, {0,0,0,0},{0,0,0,0}}};
}

/**
 * @brief 创建 3x3 单位矩阵
 * @return 3x3 单位矩阵
 */
inline Mat3 mat3_identity() {
    return {{{1,0,0}, {0,1,0}, {0,0,1}}};
}

/**
 * @brief 创建 4x4 单位矩阵
 * @return 4x4 单位矩阵
 */
inline Mat4 mat4_identity() {
    return {{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}}};
}

/**
 * @brief 计算两个三维向量的叉积
 * @param a 第一个向量
 * @param b 第二个向量
 * @return a × b
 */
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a[1]*b[2] - a[2]*b[1],  // i: a_y*b_z - a_z*b_y
        a[2]*b[0] - a[0]*b[2],  // j: a_z*b_x - a_x*b_z
        a[0]*b[1] - a[1]*b[0]   // k: a_x*b_y - a_y*b_x
    };
}

/**
 * @brief 向量标量乘法
 * @param s 标量
 * @param v 向量
 * @return s * v
 */
inline Vec3 scale(scalar s, const Vec3& v) {
    return {s*v[0], s*v[1], s*v[2]};
}

/**
 * @brief 矩阵向量乘法
 * @param M 3x3 矩阵
 * @param v 3维向量
 * @return M * v
 */
inline Vec3 mat3_mul_vec3(const Mat3& M, const Vec3& v) {
    Vec3 out{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out[i] += M[i][j] * v[j];
    return out;
}

/**
 * @brief 矩阵乘法
 * @param A 3x3 矩阵
 * @param B 3x3 矩阵
 * @return A * B
 */
inline Mat3 mat3_mul(const Mat3& A, const Mat3& B) {
    Mat3 out{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                out[i][j] += A[i][k] * B[k][j];
    return out;
}

/**
 * @brief 矩阵乘法
 * @param A 4x4 矩阵
 * @param B 4x4 矩阵
 * @return A * B
 */
inline Mat4 mat4_mul(const Mat4& A, const Mat4& B) {
    Mat4 out{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                out[i][j] += A[i][k] * B[k][j];
    return out;
}

/**
 * @brief 矩阵标量乘法
 * @param s 标量
 * @param M 3x3 矩阵
 * @return s * M
 */
inline Mat3 mat3_scale(scalar s, const Mat3& M) {
    Mat3 out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out[i][j] = s * M[i][j];
    return out;
}

/**
 * @brief 矩阵加法
 * @param A 3x3 矩阵
 * @param B 3x3 矩阵
 * @return A + B
 */
inline Mat3 mat3_add(const Mat3& A, const Mat3& B) {
    Mat3 out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out[i][j] = A[i][j] + B[i][j];
    return out;
}

/**
 * @brief 向量到反对称矩阵的映射（hat算子）
 * 
 * 将三维向量 ω = [ω_x, ω_y, ω_z] 映射为反对称矩阵：
 *   [  0  -ω_z  ω_y ]
 *   [ ω_z   0  -ω_x ]
 *   [-ω_y  ω_x   0  ]
 * 
 * @param v 三维向量
 * @return 对应的反对称矩阵
 */
inline Mat3 hat(const Vec3& v) {
    return {{
        {{0,    -v[2], v[1] }},  // 第一行
        {{v[2], 0,     -v[0]}},  // 第二行
        {{-v[1], v[0], 0    }}   // 第三行
    }};
}

/**
 * @brief 反对称矩阵到向量的映射（vee算子）
 * 
 * 将反对称矩阵映射回三维向量：
 *   [  0  -ω_z  ω_y ]  -> [ω_x, ω_y, ω_z]
 *   [ ω_z   0  -ω_x ]
 *   [-ω_y  ω_x   0  ]
 * 
 * @param M 反对称矩阵
 * @return 对应的三维向量
 */
inline Vec3 vee(const Mat3& M) {
    return {M[2][1],  // ω_x = M_{32}
            M[0][2],  // ω_y = M_{13}
            M[1][0]}; // ω_z = M_{21}
}

/**
 * @brief 两个4*4矩阵的范数
 * ∑（mat1[i][j] - mat2[i][j]）^2
 * @return 范数值
 */
inline double matrixDistance(const Mat4& mat1, const Mat4& mat2) {
    double sum = 0.0;

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            double diff = mat1[i][j] - mat2[i][j];
            sum += diff * diff;  // 平方
        }
    }

    return std::sqrt(sum);  // 开根号
}

/**
 * @brief 4×4 齐次变换矩阵求逆
 * @details 利用 SE(3) 结构 [R, t; 0, 1] 的逆为 [R^T, -R^T·t; 0, 1]
 * @param[in] m 输入的 4×4 齐次变换矩阵
 * @return 求逆后的 4×4 齐次变换矩阵
 */
inline Mat4 mat4_inverse(const Mat4& m) {
    Mat4 inv = mat4_identity();
    // R^T 部分
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            inv[i][j] = m[j][i];
        }
    }
    // -R^T · t 部分
    for (int i = 0; i < 3; ++i) {
        inv[i][3] = -(m[0][i] * m[0][3] + m[1][i] * m[1][3] + m[2][i] * m[2][3]);
    }
    inv[3][0] = inv[3][1] = inv[3][2] = 0.0;
    inv[3][3] = 1.0;
    return inv;
}

/**
 * @brief 4×4 矩阵乘以 4 维向量
 * @param[in] m 4×4 矩阵
 * @param[in] v 4 维向量
 * @return m · v
 */
inline Vec4 mat4_mul_vec4(const Mat4& m, const Vec4& v) {
    Vec4 out{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i] += m[i][j] * v[j];
        }
    }
    return out;
}



} // namespace utils


// ==================== 基础运算类 ====================

/**
 * @brief 生成零旋量
 * 
 * 返回角速度和线速度均为零的旋量
 */
struct Zero {
    Twist operator()() const { 
        return Twist(); 
    }
};

/**
 * @brief 旋量加法
 * 
 * 计算两个旋量的和：ξ₃ = ξ₁ + ξ₂
 * 
 * 结果旋量的角速度 ω₃ = ω₁ + ω₂
 *         线速度 v₃ = v₁ + v₂
 */
struct Add {
    Twist operator()(const Twist& a, const Twist& b) const {
        Vec6 out{};
        for (int i = 0; i < 6; ++i) 
            out[i] = a.data[i] + b.data[i];
        return Twist(out);
    }
};

/**
 * @brief 旋量标量乘法
 * 
 * 计算旋量的标量倍：ξ' = s * ξ
 * 
 * 结果旋量的角速度 ω' = s * ω
 *         线速度 v' = s * v
 * 
 * @param s 标量
 * @param xi 旋量
 * @return s * xi
 */
struct Scale {
    Twist operator()(scalar s, const Twist& xi) const {
        Vec6 out{};
        for (int i = 0; i < 6; ++i) 
            out[i] = s * xi.data[i];
        return Twist(out);
    }
};

/**
 * @brief 李括号运算
 * 
 * 计算两个旋量的李括号：[ξ₁, ξ₂] = [ω₁×ω₂, ω₁×v₂ - ω₂×v₁]
 * 
 * 李括号是 se(3) 上的二元运算，满足：
 *   [ξ₁, ξ₂] ∈ se(3)
 *   双线性性
 *   反对称性：[ξ₁, ξ₂] = -[ξ₂, ξ₁]
 *   Jacobi恒等式：[ξ₁,[ξ₂,ξ₃]] + [ξ₂,[ξ₃,ξ₁]] + [ξ₃,[ξ₁,ξ₂]] = 0
 */
struct LieBracket {
    Twist operator()(const Twist& a, const Twist& b) const {
        const Vec3 w1 = a.omega();
        const Vec3 v1 = a.vel();
        const Vec3 w2 = b.omega();
        const Vec3 v2 = b.vel();

        // [a,b] = [w1×w2; w1×v2 - w2×v1]
        const Vec3 w_out = utils::cross(w1, w2);
        Vec3 v_out = utils::cross(w1, v2);
        const Vec3 tmp = utils::cross(w2, v1);
        for (int i = 0; i < 3; ++i) 
            v_out[i] -= tmp[i];

        Vec6 out;
        std::copy(w_out.begin(), w_out.end(), out.begin());
        std::copy(v_out.begin(), v_out.end(), out.begin() + 3);
        return Twist(out);
    }
};

/**
 * @brief 指数映射：se(3) → SE(3)
 * 
 * 将 se(3) 中的旋量 ξ = [ω, v] 映射到 SE(3) 中的刚体变换矩阵 T。
 * 
 * 变换矩阵 T 的形式为：
 *   [ R   t ]
 *   [ 0   1 ]
 * 
 * 其中 R 是旋转矩阵，t 是平移向量。
 * 
 * 计算公式：
 *   R = exp(ω̂) = I + sinθ·ω̂/θ + (1 - cosθ)·(ω̂/θ)²
 *   V = I + (1 - cosθ)/θ²·ω̂ + (θ - sinθ)/θ³·ω̂²
 *   t = V·v
 * 
 * 当 θ = ‖ω‖ → 0 时，使用极限值避免数值不稳定。
 */
struct Exp {
    Mat4 operator()(const Twist& xi) const {
        const Vec3& w = xi.omega();  // 角速度分量
        const Vec3& v = xi.vel();    // 线速度分量
        
        // 计算旋转角度 θ = ‖ω‖
        const scalar theta_sq = w[0]*w[0] + w[1]*w[1] + w[2]*w[2];
        const scalar theta = std::sqrt(theta_sq);
        
        // 处理小角度情况（θ ≈ 0）
        if (theta < std::numeric_limits<scalar>::epsilon()) {
            Mat4 T = utils::mat4_identity();
            // 当旋转角度为零时，变换矩阵只有平移部分
            for (int i = 0; i < 3; ++i) 
                T[i][3] = v[i];
            return T;
        }
        
        // 计算旋转轴 k = ω/θ
        const Vec3 k = utils::scale(1.0/theta, w);
        const Mat3 k_hat = utils::hat(k);       // k 的反对称矩阵
        const Mat3 k_hat_sq = utils::mat3_mul(k_hat, k_hat); // k_hat²
        
        // 计算 sinθ 和 cosθ
        const scalar sin_th = std::sin(theta);
        const scalar cos_th = std::cos(theta);
        
        // 使用 Rodrigues 公式计算旋转矩阵 R
        // R = I + sinθ·k_hat + (1 - cosθ)·k_hat²
        Mat3 R = utils::mat3_identity();
        R = utils::mat3_add(R, utils::mat3_scale(sin_th, k_hat));
        R = utils::mat3_add(R, utils::mat3_scale(1.0 - cos_th, k_hat_sq));
        
        // 计算左雅可比矩阵 V
        // V = I + (1 - cosθ)/θ²·ω̂ + (θ - sinθ)/θ³·ω̂²
        const scalar a = (1 - cos_th) / theta_sq;
        const scalar b = (theta - sin_th) / (theta_sq * theta);
        
        Mat3 V = utils::mat3_identity();
        V = utils::mat3_add(V, utils::mat3_scale(a, utils::hat(w)));
        V = utils::mat3_add(V, utils::mat3_scale(b, utils::mat3_mul(utils::hat(w), utils::hat(w))));
        
        // 计算平移部分 t = V·v
        const Vec3 t = utils::mat3_mul_vec3(V, v);
        
        // 构造齐次变换矩阵 T = [R t; 0 1]
        Mat4 T = utils::mat4_identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) 
                T[i][j] = R[i][j];  // 旋转部分
            T[i][3] = t[i];         // 平移部分
        }
        return T;
    }
};

/**
 * @brief 对数映射：SE(3) → se(3)
 * 
 * 将 SE(3) 中的刚体变换矩阵 T 映射回 se(3) 中的旋量 ξ。
 * 
 * 计算步骤：
 *   1. 从 T 中提取旋转矩阵 R 和平移向量 t
 *   2. 通过 R 的迹计算旋转角度 θ：θ = acos((tr(R)-1)/2)
 *   3. 从 R 提取旋转轴 ω
 *   4. 计算左雅可比矩阵的逆 V⁻¹
 *   5. 计算线速度 v = V⁻¹·t
 *   6. 组合旋量 ξ = [ω; v]
 * 
 * 当 θ ≈ 0 时，使用极限值避免数值不稳定。
 */
struct Log {
    Twist operator()(const Mat4& T) const {
        // 提取旋转矩阵 R 和平移向量 t
        Mat3 R;
        Vec3 t;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) 
                R[i][j] = T[i][j];  // 旋转部分
            t[i] = T[i][3];         // 平移部分
        }
        
        // 计算旋转角度 θ = acos((tr(R)-1)/2)
        const scalar trace = R[0][0] + R[1][1] + R[2][2];
        const scalar theta = std::acos(
            std::max<scalar>(-1.0, std::min<scalar>(1.0, (trace - 1.0) * 0.5)));
        
        // 处理零旋转情况（θ ≈ 0）
        if (theta < std::numeric_limits<scalar>::epsilon()) {
            // 当旋转角度为零时，旋量的角速度为零，线速度为平移向量
            return Twist({0, 0, 0, t[0], t[1], t[2]});
        }
        
        // 计算旋转轴 ω（未归一化）
        Vec3 w = {
            R[2][1] - R[1][2],  // R_{32} - R_{23}
            R[0][2] - R[2][0],  // R_{13} - R_{31}
            R[1][0] - R[0][1]   // R_{21} - R_{12}
        };
        
        // 归一化旋转轴并缩放为实际角速度
        w = utils::scale(theta / (2.0 * std::sin(theta)), w);
        
        // 计算 ω 的反对称矩阵
        const Mat3 w_hat = utils::hat(w);
        const Mat3 w_hat_sq = utils::mat3_mul(w_hat, w_hat);
        const scalar theta_sq = theta * theta;
        
        // 计算逆左雅可比矩阵 V⁻¹
        // V⁻¹ = I - 0.5·ω̂ + k·ω̂²
        // 其中 k = (1 - 0.5·θ·cot(θ/2)) / θ²
        const scalar cot_half_theta = 1.0 / std::tan(theta/2.0);
        const scalar k = (1.0 - 0.5*theta*cot_half_theta) / theta_sq;
        
        Mat3 Vinv = utils::mat3_identity();
        Vinv = utils::mat3_add(Vinv, utils::mat3_scale(-0.5, w_hat));
        Vinv = utils::mat3_add(Vinv, utils::mat3_scale(k, w_hat_sq));
        
        // 计算线速度 v = V⁻¹·t
        const Vec3 v = utils::mat3_mul_vec3(Vinv, t);
        
        // 组合为旋量 ξ = [ω; v]
        return Twist({w[0], w[1], w[2], v[0], v[1], v[2]});
    }
};

/**
 * @brief 伴随映射 Ad_T：se(3) → se(3)
 * 
 * 对于给定的 SE(3) 变换 T，其伴随映射是一个 6×6 矩阵 Ad_T，
 * 它将一个坐标系下的旋量变换到另一个坐标系下。
 * 
 * 伴随矩阵的形式为：
 *   Ad_T = [ R       0 ]
 *          [ t̂R      R ]
 * 
 * 其中 R 是旋转矩阵，t 是平移向量，t̂ 是 t 的反对称矩阵。
 * 
 * 伴随映射满足：Ad_T·ξ = (T·exp(ξ)·T⁻¹)^∨
 */
struct Adjoint {
    Mat6 operator()(const Mat4& T) const {
        // 提取旋转矩阵 R 和平移向量 t
        Mat3 R;
        Vec3 t;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) 
                R[i][j] = T[i][j];  // 旋转部分
            t[i] = T[i][3];         // 平移部分
        }
        
        // 计算 t 的反对称矩阵 t̂
        const Mat3 t_hat = utils::hat(t);
        // 计算 t̂·R
        const Mat3 tR = utils::mat3_mul(t_hat, R);
        
        // 构造 6x6 伴随矩阵 Ad
        Mat6 Ad = {};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // 左上块: R
                Ad[i][j] = R[i][j];
                
                // 右上块: 0
                Ad[i][j+3] = 0.0;
                
                // 左下块: t̂·R
                Ad[i+3][j] = tR[i][j];
                
                // 右下块: R
                Ad[i+3][j+3] = R[i][j];
            }
        }
        return Ad;
    }
};

} // namespace se3_twist

#endif // SE3_TWIST_OPS_HPP













/*    

理解

SE(3) 是 “刚体的位置和朝向”（旋转+平移，李群），
se(3) 是 “刚体的角速度和线速度”（6 维向量，李代数）。
两者通过 指数/对数映射 互相转换，就像角度 ↔ 旋转矩阵。

旋量screw / twist是 se(3) 的物理化身，
而 se(3) 又是 SE(3) 的“瞬时速度版本”；


se(3) = ℝ⁶ = { ω , v }   角速度 3 + 线速度 3
它有两种“原子”数据，六种“原子”运算。
──────────────────
一、基础量2 个
1.旋量分量 ω ∈ ℝ³    (角速度，对应 se(3) 部分)
2.平移分量 v ∈ ℝ³    (线速度，对应平移部分)
3.写成 6 维向量: ξ = [ ωᵀ , vᵀ ]ᵀ ∈ ℝ⁶
──────────────────
二、基础运算6 个
1.向量加法： ξ₁ + ξ₂ = [ ω₁+ω₂ , v₁+v₂ ]   (李代数是向量空间)

2.标量乘法： α ξ = [ α ω , α v ]           (同样向量空间性质)

3.李括号Lie bracket： [ ξ₁ , ξ₂ ] = [ ω₁×ω₂ , ω₁×v₂ − ω₂×v₁ ] ∈ se(3) (封闭在 se(3) 内)

4.指数映射 exp : se(3) → SE(3)
 exp(ξ) = [ R  Vv
            0  1 ]                        (得到 4×4 齐次矩阵)
 其中 R = Exp_so(3)(ω)                     (旋转矩阵)
      V = I + (1−cosθ)/θ²  ω̂ + (θ−sinθ)/θ³  ω̂²   (θ=‖ω‖)

5.对数映射 log : SE(3) → se(3)
     log(T) = [ ω , V⁻¹ t ]             (逆运算)

6.伴随映射 Ad : SE(3) → GL(6)
     Ad_T = [ R  0
             t̂R  R ]                    (6×6 矩阵)
     满足 Ad_T ξ = (T exp(ξ) T⁻¹)̂       (共轭作用)
──────────────────
三、常用辅助
• 提升算子 (ω,v) → 4×4 矩阵
   ξ̂ = [ ω̂  v
         0  0 ]                        (用于 exp 的泰勒展开)
• 降算子 vee 把 4×4 矩阵投影回 6 维向量

*/















