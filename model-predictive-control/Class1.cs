using System;
using MathNet.Numerics.LinearAlgebra;

namespace ModelPredictiveControl
{
    public class OutputVariableScopes
    {
        public Vector<double> LowBound { get; }
        public Vector<double> UpBound { get; }

        /// <summary>
        /// 构造函数：初始化输出变量边界
        /// </summary>
        /// <param name="lb">下界向量（允许负无穷）</param>
        /// <param name="ub">上界向量（允许正无穷）</param>
        public OutputVariableScopes(Vector<double> lb, Vector<double> ub)
        {
            LowBound = lb ?? throw new ArgumentNullException(nameof(lb), "下界向量不可为 null");
            UpBound = ub ?? throw new ArgumentNullException(nameof(ub), "上界向量不可为 null");
            _ = DimCheck(); // 构造时自动检查边界有效性
        }

        /// <summary>
        /// 边界值有效性检查（支持无穷大逻辑）
        /// </summary>
        /// <exception cref="ArgumentException">违反边界规则时抛出</exception>
        public bool DimCheck()
        {
            // 1. 维度一致性检查
            if (LowBound.Count != UpBound.Count)
                throw new ArgumentException(
                    $"边界维度不匹配。下界维度：{LowBound.Count}，上界维度：{UpBound.Count}"
                );

            // 2. 逐元素边界规则校验
            for (int i = 0; i < LowBound.Count; i++)
            {
                double lb = LowBound[i];
                double ub = UpBound[i];

                // 规则1：两端均为无穷 → 非法
                if (double.IsNegativeInfinity(lb) && double.IsPositiveInfinity(ub))
                    throw new ArgumentException(
                        $"索引 {i} 处边界无效：上下界不能同时为无穷大"
                    );

                // 规则2：下界大于上界 → 非法（非无穷比较）
                if (!double.IsInfinity(lb) && !double.IsInfinity(ub) && lb > ub)
                    throw new ArgumentException(
                        $"索引 {i} 处边界无效：下界 ({lb}) 必须 ≤ 上界 ({ub})"
                    );

                // 规则3：下界正无穷 → 非法（无数值能大于正无穷）
                if (double.IsPositiveInfinity(lb))
                    throw new ArgumentException(
                        $"索引 {i} 处边界无效：下界不能为 +∞"
                    );

                // 规则4：上界负无穷 → 非法（无数值能小于负无穷）
                if (double.IsNegativeInfinity(ub))
                    throw new ArgumentException(
                        $"索引 {i} 处边界无效：上界不能为 -∞"
                    );
            }
            return true;
        }
    }

    // 控制变量的范围（绝对范围）
    public class ManipulatedVariableScopes
    {
        public Vector<double> LowBound { get; set; }
        public Vector<double> UpBound { get; set; }

        public ManipulatedVariableScopes(Vector<double> lb, Vector<double> ub)
        {
            LowBound = lb;
            UpBound = ub;
            _ = DimChek(); // 构造时自动检查维度
        }

        /// <summary>
        /// 检查边界向量的维度和数值有效性
        /// </summary>
        public bool DimChek()
        {
            // 1. 检查是否为 null
            if (LowBound == null || UpBound == null)
                throw new ArgumentNullException("边界向量不可为 null");

            // 2. 检查维度一致性
            if (LowBound.Count != UpBound.Count)
                throw new ArgumentException(
                    $"边界向量维度不一致。下界维度：{LowBound.Count}，上界维度：{UpBound.Count}"
                );

            // 3. 检查数值有效性：下界 ≤ 上界
            for (int i = 0; i < LowBound.Count; i++)
            {
                if (LowBound[i] > UpBound[i])
                    throw new ArgumentException(
                        $"索引 {i} 处边界值无效：下界 ({LowBound[i]}) 必须 ≤ 上界 ({UpBound[i]})"
                    );
            }

            return true;
        }
    }

    // 控制变量变化的范围（增量范围）
    public class ManipulatedVariableIncScopes
    {
        public Vector<double> LowBound { get; set; } // 增量下界（通常为负值）
        public Vector<double> UpBound { get; set; }   // 增量上界（通常为正值）

        public ManipulatedVariableIncScopes(Vector<double> lb, Vector<double> ub)
        {
            LowBound = lb;
            UpBound = ub;
            _ = DimChek(); // 构造时自动检查维度
        }

        /// <summary>
        /// 检查增量边界向量的维度和数值有效性
        /// </summary>
        public bool DimChek()
        {
            // 1. 检查是否为 null
            if (LowBound == null || UpBound == null)
                throw new ArgumentNullException("边界向量不可为 null");

            // 2. 检查维度一致性
            if (LowBound.Count != UpBound.Count)
                throw new ArgumentException(
                    $"边界向量维度不一致。下界维度：{LowBound.Count}，上界维度：{UpBound.Count}"
                );

            // 3. 检查数值有效性：下界 ≤ 上界，且下界 ≤ 0 ≤ 上界（增量范围需包含0）
            for (int i = 0; i < LowBound.Count; i++)
            {
                if (LowBound[i] > UpBound[i])
                    throw new ArgumentException(
                        $"索引 {i} 处增量边界值无效：下界 ({LowBound[i]}) 必须 ≤ 上界 ({UpBound[i]})"
                    );

                if (LowBound[i] > 0 || UpBound[i] < 0)
                    throw new ArgumentException(
                        $"索引 {i} 处增量范围必须包含 0。当前范围：[{LowBound[i]}, {UpBound[i]}]"
                    );
            }

            return true;
        }
    }

    public class Mpc
    {
        public MpcSys Sys { get; set; }

        public struct MpcInfo
        {
            public alglib.minqpreport rep;
            public double[] result;
        }

        public int PredictionHorizon { get; set; }
        public int ControlHorizon { get; set; }


        public ManipulatedVariableScopes MVConstraints { get; set; } // 输入约束
        public OutputVariableScopes OVConstraints { get; set; } // 输出约束
        public ManipulatedVariableIncScopes MVIncConstraints { get; set; } // 增量输入约束（增量式MPC专用）


        public Vector<double> ManipulatedVariablesWeight { get; set; } // 输入变量权重
        public Vector<double> OutputVariablesWeight { get; set; } // 输出变量权重
        public Vector<double> OutputVariablesFinalWeight { get; set; } // 输出变量终端约束权重


        public Matrix<double> Q { get; set; } // 输出权重矩阵
        public Matrix<double> R { get; set; } // 输入权重矩阵
        public Matrix<double> F { get; set; } // 终端约束权重矩阵


        public Matrix<double> Sx { get; set; } // MPC模型预测状态转移矩阵 Sx
        public Matrix<double> Su { get; set; } // MPC模型预测控制输入矩阵 Su
        public Matrix<double> Su_delta { get; set; } // 增量式MPC模型预测控制输入矩阵 Su_delta


        public Matrix<double> OVScalefactor { get; set; } // 输出变量权重归一化因子(对角矩阵) 1/s_Q
        public Matrix<double> OVFScalefactor { get; set; } // 输出变量终端约束权重归一化因子(对角矩阵) 1/s_F
        public Matrix<double> MVScalefactor { get; set; } // 输入变量权重归一化因子(对角矩阵) 1/s_R
        public Matrix<double> MVIncScalefactor { get; set; } // (增量MPC)增量输入变量权重归一化因子(对角矩阵) 1/s_R


        public Matrix<double> H_qpsolver { get; set; } // QP求解器的H矩阵
        public Matrix<double> f_qpsolver { get; set; } // QP求解器的F矩阵


        public double SolutionTolerance { get; set; } = 1e-6; // QP求解容忍度


        public int ManipulatedVariablesDim => Sys?.ManipulatedVariablesDim ?? 0;
        public int OutputVariablesDim => Sys?.OutputVariablesDim ?? 0;
        public int StateVariablesDim => Sys?.StateVariablesDim ?? 0;


        public Mpc(MpcSys sys, int predictionHorizon, Vector<double> mvWeight, Vector<double> ovWeight,
            ManipulatedVariableScopes mvs, OutputVariableScopes ovs = null)
        {
            Sys = sys;
            PredictionHorizon = predictionHorizon;
            ControlHorizon = predictionHorizon;
            ManipulatedVariablesWeight = mvWeight;
            OutputVariablesWeight = ovWeight;
            MVConstraints = mvs;
            OVConstraints = ovs;

            _ = DimChek(); // 构造时自动检查维度
            InitializeWeights(); // 初始化权重矩阵
        }

        public Mpc(MpcSys sys, int predictionHorizon, Vector<double> mvWeight, Vector<double> ovWeight,
            ManipulatedVariableScopes mvs, ManipulatedVariableIncScopes mvis, OutputVariableScopes ovs = null)
        {
            Sys = sys;
            PredictionHorizon = predictionHorizon;
            ControlHorizon = predictionHorizon;
            ManipulatedVariablesWeight = mvWeight;
            OutputVariablesWeight = ovWeight;
            MVConstraints = mvs;
            OVConstraints = ovs;
            MVIncConstraints = mvis;

            _ = DimChek(); // 构造时自动检查维度
            InitializeWeights(); // 初始化权重矩阵
        }

        public Mpc(MpcSys sys, int predictionHorizon, Vector<double> mvWeight, Vector<double> ovWeight, Vector<double> ovfWeight,
            ManipulatedVariableScopes mvs, ManipulatedVariableIncScopes mvis, OutputVariableScopes ovs = null)
        {
            Sys = sys;
            PredictionHorizon = predictionHorizon;
            ControlHorizon = predictionHorizon;
            ManipulatedVariablesWeight = mvWeight;
            OutputVariablesWeight = ovWeight;
            OutputVariablesFinalWeight = ovfWeight;

            MVConstraints = mvs;
            OVConstraints = ovs;
            MVIncConstraints = mvis;

            _ = DimChek(); // 构造时自动检查维度
            InitializeWeights(); // 初始化权重矩阵
        }

        private void InitializeWeights()
        {
            // 控制变量权重归一化因子更新 1/s_R
            MVScalefactor = MVScalefactorUpdate();

            // 构建对角权重矩阵
            Q = BuildDiagonalMatrix(OutputVariablesWeight, PredictionHorizon);
            R = BuildDiagonalMatrix(ManipulatedVariablesWeight, ControlHorizon);
            F = Matrix<double>.Build.DenseOfDiagonalVector(OutputVariablesFinalWeight);

            if (MVIncConstraints != null)
            {
                // 初始化增量MPC控制输入矩阵 Su_delta
                Su_delta = CreateLTBlockMatrix(ManipulatedVariablesDim, ControlHorizon);

                // 增量输入变量权重归一化因子更新 1/s_R
                MVIncScalefactor = MVIncScalefactorUpdate();
            }
        }

        public static Matrix<double> CreateLTBlockMatrix(int m, int p)
        {
            // 计算总维度
            int totalSize = p * m;
            // 初始化全零矩阵
            var S = Matrix<double>.Build.Dense(totalSize, totalSize, 0.0);

            // 生成 m×m 单位矩阵块
            var identityBlock = Matrix<double>.Build.DenseIdentity(m);

            // 填充下三角区域
            for (int blockRow = 0; blockRow < p; blockRow++)
            {
                for (int blockCol = 0; blockCol <= blockRow; blockCol++)
                {
                    // 计算当前块在矩阵中的起始位置
                    int rowStart = blockRow * m;
                    int colStart = blockCol * m;

                    // 将单位矩阵块复制到 S 的对应位置
                    S.SetSubMatrix(rowStart, colStart, identityBlock);
                }
            }
            return S;
        }

        public void MVConstraintsUpdate(ManipulatedVariableScopes mvs)
        {
            MVConstraints = mvs ?? throw new ArgumentNullException(nameof(mvs), "控制变量范围不可为 null");
            _ = MVConstraints.DimChek(); // 更新后重新检查维度

            MVScalefactor = MVScalefactorUpdate(); // 更新后重新计算控制变量权重归一化因子
        }


        public static Matrix<double> BuildDiagonalMatrix(Vector<double> vw, int n)
        {
            // 参数校验
            if (vw == null)
                throw new ArgumentException("权重向量不可为 null", nameof(vw));
            if (n <= 0)
                throw new ArgumentException("预测或者控制时域必须为正整数", nameof(n));
            if (vw.Count == 0)
                throw new ArgumentException("权重向量不能为空", nameof(vw));

            // 计算总维度
            int weightSize = vw.Count;
            int totalSize = weightSize * n;

            Vector<double> diagonalVector = Vector<double>.Build.Dense(vw.Count * n,
                i => vw[i % vw.Count]);

            // 创建对角矩阵（非零元素仅在对角线上）[2,6](@ref)
            return Matrix<double>.Build.DenseOfDiagonalVector(diagonalVector);
        }

        /// <summary>
        /// 维度一致性检查
        /// </summary>
        public bool DimChek()
        {
            if (Sys == null)
                throw new ArgumentException("预测模型输入错误");

            // 检查预测时域有效性
            if (PredictionHorizon <= 0)
                throw new ArgumentException("预测时域必须大于0");

            // 检查输入范围约束是否有效
            if (MVConstraints == null)
                throw new ArgumentException("预测模型输入约束输入错误");

            // 检查模型输入约束维度是否等于系统输入维度ManipulatedVariablesDim
            // 检查输入约束维度是否等于系统输入维度
            if (MVConstraints.LowBound.Count != ManipulatedVariablesDim)
                throw new ArgumentException($"输入约束维度 {MVConstraints.LowBound.Count} 与系统输入维度 {ManipulatedVariablesDim} 不匹配");

            // 检查输出约束有效性（如果定义了输出约束）
            if (OVConstraints != null)
            {
                if (OVConstraints.LowBound.Count != OutputVariablesDim)
                    throw new ArgumentException($"输出约束维度 {OVConstraints.LowBound.Count} 与系统输出维度 {OutputVariablesDim} 不匹配");
            }

            return true;
        }

        // 构建Sx = [CA^1 : CA^2 : … : C*A^N]
        public static Matrix<double> BuildSx(Matrix<double> C, Matrix<double> A, int N)
        {
            // 获取矩阵维度
            int p = C.RowCount;    // C 的行数 (p)
            int n = C.ColumnCount; // C 的列数 (n)，也即 A 的阶数

            // 计算 A 的幂次 (0 到 N)
            Matrix<double>[] APowers = new Matrix<double>[N + 1];
            APowers[0] = Matrix<double>.Build.DenseIdentity(n); // A⁰ = I (单位矩阵)
            for (int i = 1; i <= N; i++)
            {
                APowers[i] = APowers[i - 1].Multiply(A); // Aⁱ = Aⁱ⁻¹ × A
            }

            // 构造 Sx 矩阵 (维度: p×N 行, n 列)
            int totalRows = p * N;
            Matrix<double> _Sx = Matrix<double>.Build.Dense(totalRows, n);

            // 填充 Sx：垂直堆叠 C×A¹, C×A², ..., C×Aᴺ
            for (int i = 1; i <= N; i++)
            {
                Matrix<double> block = C.Multiply(APowers[i]); // C × Aⁱ
                int startRow = (i - 1) * p;                   // 当前块的起始行索引
                _Sx.SetSubMatrix(startRow, 0, block);          // 将块放入 Sx 对应位置
            }

            return _Sx;
        }

        // 构造 Su=[C*A^0*B D 0 ... 0;...;C*A^(N-1)*B ... D]  D不为0或者null
        public static Matrix<double> BuildSu(Matrix<double> C, Matrix<double> A, Matrix<double> B, Matrix<double> D, int N)
        {
            // 1. 获取矩阵维度
            int p = C.RowCount;     // 输出通道数 (C的行数)
            int m = B.ColumnCount;  // 输入通道数 (B的列数)
            int n = A.RowCount;     // 系统阶数 (A的维度)

            // 2. 预计算 A 的幂次: A⁰, A¹, ..., Aᴺ
            Matrix<double>[] APowers = new Matrix<double>[N + 1];
            APowers[0] = Matrix<double>.Build.DenseIdentity(n); // A⁰ = I
            for (int i = 1; i <= N; i++)
            {
                APowers[i] = APowers[i - 1].Multiply(A);  // Aⁱ = Aⁱ⁻¹ × A
            }

            // 3. 预计算核心块：C × Aᵏ × B (k=0 到 N)
            Matrix<double>[] CAB = new Matrix<double>[N + 1];
            for (int i = 0; i <= N; i++)
            {
                CAB[i] = C.Multiply(APowers[i]).Multiply(B);
            }

            // 4. 初始化 Su 矩阵 (维度: p×N 行, m×(N+1) 列)
            int totalRows = p * N;
            int totalCols = m * (N + 1);
            Matrix<double> _Su = Matrix<double>.Build.Dense(totalRows, totalCols, 0);

            // 5. 按块填充下三角结构
            for (int i = 0; i < N; i++)
            {
                int rowStart = i * p;  // 当前块行起始位置
                int colStart = 0;      // 当前块列起始位置

                // 填充当前行的下三角块 (数量 = i+1)
                for (int j = 0; j <= i; j++)
                {
                    // 关键：CAB[i-j] = C × Aⁱ⁻ʲ × B
                    Matrix<double> block = CAB[i - j];
                    _Su.SetSubMatrix(rowStart, colStart, block);
                    colStart += block.ColumnCount;  // 移动到下一块起始列
                }

                // 每行末尾添加 D 矩阵
                _Su.SetSubMatrix(rowStart, colStart, D);
            }

            return _Su;
        }

        // 构造 Su=[C*A^0*B 0 0 ... 0;...;C*A^(N-1)*B ... C*A^0*B]  D为0或者null
        public static Matrix<double> BuildSu(Matrix<double> C, Matrix<double> A, Matrix<double> B, int N)
        {
            // 获取矩阵维度
            int p = C.RowCount;     // 输出通道数
            int n = C.ColumnCount;  // 系统阶数（A的维度）
            int m = B.ColumnCount;  // 输入通道数

            // 预计算 A 的幂次 (0 到 N)
            Matrix<double>[] APowers = new Matrix<double>[N + 1];
            APowers[0] = Matrix<double>.Build.DenseIdentity(n); // A⁰ = I
            for (int i = 1; i <= N; i++)
            {
                APowers[i] = APowers[i - 1].Multiply(A); // Aⁱ = Aⁱ⁻¹ × A
            }

            // 预计算核心块 C × Aᵏ × B
            Matrix<double>[] CAB = new Matrix<double>[N + 1];
            for (int k = 0; k <= N; k++)
            {
                CAB[k] = C.Multiply(APowers[k]).Multiply(B); // C × Aᵏ × B
            }

            // 初始化 Su 矩阵 (维度: p×N 行, m×N 列)
            int totalRows = p * N;
            int totalCols = m * N;
            Matrix<double> _Su = Matrix<double>.Build.Dense(totalRows, totalCols);

            // 构建下三角结构（D=0 的特殊形式）
            for (int i = 0; i < N; i++) // 每行对应系统输出的一步
            {
                int rowStart = i * p;   // 当前块的起始行
                int colStart = 0;       // 当前列的起始位置

                // 每行有 i+1 个非零块
                for (int j = 0; j <= i; j++)
                {
                    int k = i - j;     // A 的幂次 (从高到低)
                                       // 放置 C × Aᵏ × B 块
                    _Su.SetSubMatrix(rowStart, colStart, CAB[k]);
                    colStart += CAB[k].ColumnCount; // 移动到下一块
                }
                // 注意：此处缺少了 D 矩阵的添加！
            }
            return _Su;
        }

        // 输入权重归一化因子更新 (1/s_R更新)
        public Matrix<double> MVScalefactorUpdate()
        {
            if (MVConstraints == null) return null;
            Vector<double> temp = 1 / (MVConstraints.UpBound - MVConstraints.LowBound).PointwiseMultiply(
                MVConstraints.UpBound - MVConstraints.LowBound);
            Vector<double> diagonal = Vector<double>.Build.Dense(temp.Count * ControlHorizon,
                i => temp[i % temp.Count]);

            return Matrix<double>.Build.DiagonalOfDiagonalVector(diagonal);
        }

        // 增量MPC输入权重归一化因子更新 (1/s_R更新)
        public Matrix<double> MVIncScalefactorUpdate()
        {
            if (MVIncConstraints == null) return null;
            Vector<double> temp = 1 / (MVIncConstraints.UpBound - MVIncConstraints.LowBound).PointwiseMultiply(
                MVIncConstraints.UpBound - MVIncConstraints.LowBound);
            Vector<double> diagonal = Vector<double>.Build.Dense(temp.Count * ControlHorizon,
                i => temp[i % temp.Count]);

            return Matrix<double>.Build.DiagonalOfDiagonalVector(diagonal);
        }

        private static Vector<double> VectorCopy(Vector<double> vec, int n)
        {
            if (vec == null) return null;

            return Vector<double>.Build.Dense(vec.Count * n,
                i => vec[i % vec.Count]);
        }

        // 输出权重归一化因子更新 (1/s_Q更新)
        public Matrix<double> OVScalefactorUpdate(Vector<double> reference)
        {
            Vector<double> temp = 1 / reference.PointwiseMultiply(reference);
            Vector<double> diagonal = Vector<double>.Build.Dense(temp.Count * PredictionHorizon,
                i => temp[i % temp.Count]);

            return Matrix<double>.Build.DiagonalOfDiagonalVector(diagonal);
        }

        public Matrix<double> OVFScalefactorUpdate(Vector<double> reference)
        {
            Vector<double> temp = 1 / reference.PointwiseMultiply(reference);
            Vector<double> diagonal = Vector<double>.Build.Dense(temp.Count * 1,
                i => temp[i % temp.Count]);

            return Matrix<double>.Build.DiagonalOfDiagonalVector(diagonal);
        }

        public void MpcSysUpdate(MpcSys sys)
        {
            Sys = sys ?? throw new ArgumentNullException(nameof(sys), "MPC系统模型不能为空");
            _ = DimChek(); // 更新后重新检查维度
            InitializeWeights(); // 重新初始化权重矩阵
        }

        public void MpcSysUpdate(Matrix<double> a, Matrix<double> b, Matrix<double> c, Matrix<double> d, double ts)
        {
            Sys.A = a ?? throw new ArgumentNullException(nameof(a), "状态矩阵A不能为空");
            Sys.B = b ?? throw new ArgumentNullException(nameof(b), "输入矩阵B不能为空");
            Sys.C = c ?? throw new ArgumentNullException(nameof(c), "输出矩阵C不能为空");
            Sys.D = d; // D可以为null
            Sys.Ts = ts;
            _ = Sys.DimCheck(); // 更新后重新检查维度
            _ = DimChek(); // 更新后重新检查MPC维度
            InitializeWeights(); // 重新初始化权重矩阵
        }

        // 模型参数更新与预测矩阵更新
        public bool MpcSysUpdate(Matrix<double> a, Matrix<double> b, Matrix<double> c, Matrix<double> d = null)
        {
            try
            {
                // 更新系统模型参数 A,B,C
                if (Sys.SysUpdate(a, b, c, false))
                {
                    // 更新状态预测矩阵
                    Sx = BuildSx(Sys.C, Sys.A, PredictionHorizon);

                    // 更新控制输入矩阵
                    Su = BuildSu(Sys.C, Sys.A, Sys.B, ControlHorizon);
                }
                else return false;
            }
            catch { throw; } // 异常重新抛出
            return true;
        }

        public bool IncMpcSysUpdate(Matrix<double> a, Matrix<double> b, Matrix<double> c, Matrix<double> d = null)
        {
            try
            {
                // 更新增量式MPC系统模型参数 A,B,C
                if (Sys.SysUpdate(a, b, c))
                {
                    // 更新状态预测矩阵 Sx
                    Sx = BuildSx(Sys.C, Sys.A, PredictionHorizon);

                    // 更新控制输入矩阵 Su
                    Su = BuildSu(Sys.C, Sys.A, Sys.B, ControlHorizon);
                }
                else return false;
            }
            catch { throw; } // 异常重新抛出
            return true;
        }

        public MpcInfo MpcMove(Vector<double> msv, Vector<double> r)
        {
            MpcInfo mpcInfo;

            // 输出权重归一化因子更新
            OVScalefactor = OVScalefactorUpdate(r);

            // 计算加权矩阵
            Matrix<double> Q_scaled = Q * OVScalefactor;
            Matrix<double> R_scaled = R * MVScalefactor;

            // 零输入状态预测 Sx * x(k)
            Matrix<double> Sx_mStateVariables = Sx * msv.ToColumnMatrix();

            // 构建二次规划问题矩阵
            Vector<double> r_pred = VectorCopy(r, PredictionHorizon);
            H_qpsolver = (Su.Transpose() * Q_scaled * Su) + R_scaled;
            f_qpsolver = (Sx_mStateVariables.Transpose() - r_pred.ToRowMatrix()) * Q_scaled * Su;

            int qpDim = ManipulatedVariablesDim * ControlHorizon;

            double[,] H = H_qpsolver.ToArray();
            double[] f = f_qpsolver.Row(0).ToArray();

            // 构建输入约束
            double[] lb = VectorCopy(MVConstraints.LowBound, ControlHorizon).ToArray();
            double[] ub = VectorCopy(MVConstraints.UpBound, ControlHorizon).ToArray();

            if (OVConstraints != null)
            {
                // 构建输出约束
                Matrix<double> al_matrix = VectorCopy(OVConstraints.LowBound, PredictionHorizon).ToColumnMatrix() - Sx_mStateVariables;
                Matrix<double> au_matrix = VectorCopy(OVConstraints.UpBound, PredictionHorizon).ToColumnMatrix() - Sx_mStateVariables;

                double[] al = al_matrix.Column(0).ToArray();
                double[] au = au_matrix.Column(0).ToArray();

                double[,] ac = Su.ToArray();

                // 调用QP求解器
                mpcInfo = QP(qpDim, H, f, lb, ub, ac, al, au);


                //Console.WriteLine($"H = {FormatMatrix(H_qpsolver)};");
                //Console.WriteLine($"f = {FormatVector(f_qpsolver.Row(0))};");
                //Console.WriteLine($"lb = {FormatVector(VectorCopy(MVConstraints.LowBound, ControlHorizon))};");
                //Console.WriteLine($"ub = {FormatVector(VectorCopy(MVConstraints.UpBound, ControlHorizon))};");
                //Console.WriteLine($"ac = {FormatMatrix(Su)};");
                //Console.WriteLine($"al = {FormatVector(al_matrix.Column(0))};");
                //Console.WriteLine($"au = {FormatVector(au_matrix.Column(0))};");

                //Console.WriteLine($"result = {FormatVector(Vector<double>.Build.Dense(mpcInfo.result))};");
                //Console.WriteLine($"msv[0] = {msv[0]};");

                //Console.WriteLine("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
            }
            else
            {
                mpcInfo = QP(qpDim, H, f, lb, ub);
            }
            return mpcInfo;
        }

        public MpcInfo FMpcMove(Vector<double> msv, Vector<double> r)
        {
            MpcInfo mpcInfo;

            // 输出权重归一化因子更新
            OVScalefactor = OVScalefactorUpdate(r);
            OVFScalefactor = OVFScalefactorUpdate(r);


            // 计算加权矩阵
            Matrix<double> Q_scaled = Q * OVScalefactor;
            Matrix<double> R_scaled = R * MVScalefactor;
            Matrix<double> F_scaled = F * OVFScalefactor;

            // 构建
            Matrix<double> Sx_final = Sx.SubMatrix(OutputVariablesDim * (PredictionHorizon - 1),
                OutputVariablesDim, 0, Sx.ColumnCount);
            // 
            Matrix<double> Su_final = Su.SubMatrix(OutputVariablesDim * (ControlHorizon - 1),
                OutputVariablesDim, 0, Su.ColumnCount);

            // 零输入状态预测 Sx * x(k)
            Matrix<double> Sx_mStateVariables = Sx * msv.ToColumnMatrix();
            Matrix<double> Sx_final_mStateVariables = Sx_final * msv.ToColumnMatrix();

            // 构建二次规划问题矩阵
            Vector<double> r_pred = VectorCopy(r, PredictionHorizon);
            H_qpsolver = (Su.Transpose() * Q_scaled * Su) + (Su_final.Transpose() * F_scaled * Su_final) + R_scaled;
            f_qpsolver = (Sx_mStateVariables.Transpose() - r_pred.ToRowMatrix()) * Q_scaled * Su +
                         (Sx_final_mStateVariables.Transpose() - r.ToRowMatrix()) * F_scaled * Su_final;

            int qpDim = ManipulatedVariablesDim * ControlHorizon;

            double[,] H = H_qpsolver.ToArray();
            double[] f = f_qpsolver.Row(0).ToArray();

            // 构建输入约束
            double[] lb = VectorCopy(MVConstraints.LowBound, ControlHorizon).ToArray();
            double[] ub = VectorCopy(MVConstraints.UpBound, ControlHorizon).ToArray();

            if (OVConstraints != null)
            {
                // 构建输出约束
                Matrix<double> al_matrix = VectorCopy(OVConstraints.LowBound, PredictionHorizon).ToColumnMatrix() - Sx_mStateVariables;
                Matrix<double> au_matrix = VectorCopy(OVConstraints.UpBound, PredictionHorizon).ToColumnMatrix() - Sx_mStateVariables;

                double[] al = al_matrix.Column(0).ToArray();
                double[] au = au_matrix.Column(0).ToArray();

                double[,] ac = Su.ToArray();

                // 调用QP求解器
                mpcInfo = QP(qpDim, H, f, lb, ub, ac, al, au);


                //Console.WriteLine($"H = {FormatMatrix(H_qpsolver)};");
                //Console.WriteLine($"f = {FormatVector(f_qpsolver.Row(0))};");
                //Console.WriteLine($"lb = {FormatVector(VectorCopy(MVConstraints.LowBound, ControlHorizon))};");
                //Console.WriteLine($"ub = {FormatVector(VectorCopy(MVConstraints.UpBound, ControlHorizon))};");
                //Console.WriteLine($"ac = {FormatMatrix(Su)};");
                //Console.WriteLine($"al = {FormatVector(al_matrix.Column(0))};");
                //Console.WriteLine($"au = {FormatVector(au_matrix.Column(0))};");

                //Console.WriteLine($"result = {FormatVector(Vector<double>.Build.Dense(mpcInfo.result))};");
                //Console.WriteLine($"msv[0] = {msv[0]};");

                //Console.WriteLine("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
            }
            else
            {
                mpcInfo = QP(qpDim, H, f, lb, ub);
            }
            return mpcInfo;
        }

        public MpcInfo IncMpcMove(Vector<double> msv, Vector<double> r)
        {
            MpcInfo mpcInfo = new MpcInfo
            {
                rep = new alglib.minqpreport(),
                result = null
            };

            // 输出权重归一化因子更新
            OVScalefactor = OVScalefactorUpdate(r);

            // 计算加权矩阵
            Matrix<double> Q_scaled = Q * OVScalefactor;
            Matrix<double> R_scaled = R * MVIncScalefactor; // 采用增量输入权重归一化因子

            // 零输入状态预测 Sx * x(k)
            Matrix<double> Sx_mStateVariables = Sx * msv.ToColumnMatrix();

            // 构建二次规划问题矩阵
            Vector<double> r_pred = VectorCopy(r, PredictionHorizon);
            H_qpsolver = (Su.Transpose() * Q_scaled * Su) + R_scaled;
            f_qpsolver = (Sx_mStateVariables.Transpose() - r_pred.ToRowMatrix()) * Q_scaled * Su;

            int qpDim = ManipulatedVariablesDim * ControlHorizon;

            double[,] H = H_qpsolver.ToArray();
            double[] f = f_qpsolver.Row(0).ToArray();

            // 增量式MPC的输入约束
            double[] lb = VectorCopy(MVIncConstraints.LowBound, ControlHorizon).ToArray();
            double[] ub = VectorCopy(MVIncConstraints.UpBound, ControlHorizon).ToArray();

            // 增量式MPC的输出约束
            Matrix<double> al_matrix = VectorCopy(MVConstraints.LowBound - msv[msv.Count - 1], PredictionHorizon).ToColumnMatrix();
            Matrix<double> au_matrix = VectorCopy(MVConstraints.UpBound - msv[msv.Count - 1], PredictionHorizon).ToColumnMatrix();

            Matrix<double> ac_matrix = Su_delta;

            if (OVConstraints != null)
            {
                // 构建输出约束
                Matrix<double> al_matrix_ov = VectorCopy(OVConstraints.LowBound, PredictionHorizon).ToColumnMatrix() - Sx_mStateVariables;
                Matrix<double> au_matrix_ov = VectorCopy(OVConstraints.UpBound, PredictionHorizon).ToColumnMatrix() - Sx_mStateVariables;

                // 垂直拼接输出约束
                al_matrix = ConcatMatrixColumly(al_matrix, al_matrix_ov);
                au_matrix = ConcatMatrixColumly(au_matrix, au_matrix_ov);

                // 垂直拼接ac
                ac_matrix = ConcatMatrixColumly(ac_matrix, Su);
            }

            double[] al = al_matrix.Column(0).ToArray();
            double[] au = au_matrix.Column(0).ToArray();

            double[,] ac = ac_matrix.ToArray();

            // 调用QP求解器
            mpcInfo = QP(qpDim, H, f, lb, ub, ac, al, au);


            return mpcInfo;
        }

        public MpcInfo QP(int qpDim, double[,] H, double[] f, double[] lb, double[] ub)
        {
            bool isupper = true;
            double[] result;

            double[] s = Vector<double>.Build.Dense(qpDim, 1).ToArray();

            alglib.minqpstate state;
            alglib.minqpreport rep;

            // create solver, set quadratic/linear terms
            alglib.minqpcreate(qpDim, out state);
            alglib.minqpsetquadraticterm(state, H, isupper);
            alglib.minqpsetlinearterm(state, f);

            // alglib.minqpsetlc2dense(state, ac, al, au);

            alglib.minqpsetbc(state, lb, ub);

            alglib.minqpsetscale(state, s);
            //alglib.minqpsetscaleautodiag(state);

            alglib.minqpsetalgosparseipm(state, 0.0);
            alglib.minqpoptimize(state);
            alglib.minqpresults(state, out result, out rep);

            return new MpcInfo
            {
                rep = rep,
                result = result
            };
        }

        public MpcInfo QP(int qpDim, double[,] H, double[] f, double[] lb, double[] ub, double[,] ac, double[] al, double[] au)
        {
            bool isupper = true;
            double[] result;

            double[] s = Vector<double>.Build.Dense(qpDim, 1).ToArray();

            alglib.minqpstate state;
            alglib.minqpreport rep;

            // create solver, set quadratic/linear terms
            alglib.minqpcreate(qpDim, out state);
            alglib.minqpsetquadraticterm(state, H, isupper);
            alglib.minqpsetlinearterm(state, f);

            alglib.minqpsetlc2dense(state, ac, al, au);

            alglib.minqpsetbc(state, lb, ub);

            alglib.minqpsetscale(state, s);
            //alglib.minqpsetscaleautodiag(state);

            alglib.minqpsetalgosparseipm(state, 0.0);
            alglib.minqpoptimize(state);
            alglib.minqpresults(state, out result, out rep);

            return new MpcInfo
            {
                rep = rep,
                result = result
            };
        }

        public static Vector<double> VecCumulativeSum(Vector<double> vec)
        {
            if (vec == null)
            {
                throw new ArgumentNullException(nameof(vec), "向量不能为空");
            }

            int n = vec.Count;
            double[] result = new double[n];
            double sum = 0;

            for (int i = 0; i < n; i++)
            {
                sum += vec[i];
                result[i] = sum;
            }

            return Vector<double>.Build.Dense(result);
        }

        // 格式化矩阵为 MATLAB 风格的字符串
        public static string FormatMatrix(Matrix<double> matrix, int decimalPlaces = 6)
        {
            int rows = matrix.RowCount;
            int cols = matrix.ColumnCount;
            string result = "[";
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    result += matrix[i, j].ToString("F" + decimalPlaces);
                    if (j < cols - 1) result += " ";
                }
                if (i < rows - 1) result += "; ";
            }
            result += "]";
            return result;
        }

        // 格式化向量为 MATLAB 风格的字符串
        public static string FormatVector(Vector<double> vector, int decimalPlaces = 6)
        {
            string result = "[";
            for (int i = 0; i < vector.Count; i++)
            {
                result += vector[i].ToString("F" + decimalPlaces);
                if (i < vector.Count - 1) result += "; ";
            }
            result += "]";
            return result;
        }

        // 垂直拼接两个矩阵
        public static Matrix<double> ConcatMatrixColumly(Matrix<double> top, Matrix<double> bottom)
        {
            try
            {
                // 1. 参数基础验证
                if (top == null || bottom == null)
                {
                    string errorMsg = "输入矩阵不能为null";
                    throw new ArgumentNullException(errorMsg); // 明确异常类型[2](@ref)
                }

                // 2. 维度校验（核心约束）
                if (top.ColumnCount != bottom.ColumnCount)
                {
                    string errorMsg = $"列数不匹配: top={top.ColumnCount}, bottom={bottom.ColumnCount}";

                    throw new ArgumentException(errorMsg); // 提供具体参数值[5](@ref)
                }

                // 3. 执行拼接
                return Matrix<double>.Build.DenseOfMatrixArray(
                    new[,] { { top }, { bottom } }
                );
            }
            catch (ArgumentNullException)
            {
                throw; // 重新抛出原始异常[7](@ref)
            }
            catch (ArgumentException ex)
            {
                throw new InvalidOperationException("垂直拼接失败：请检查矩阵维度", ex); // 包裹原生异常[6](@ref)
            }
            catch (Exception ex) // 兜底未知错误[1](@ref)
            {
                throw new InvalidOperationException($"矩阵拼接发生意外错误: {ex.Message}");
            }
        }
    }

    public class MpcSys
    {
        public Matrix<double> A { get; set; }
        public Matrix<double> B { get; set; }
        public Matrix<double> C { get; set; }
        public Matrix<double> D { get; set; }
        public double Ts { get; set; }

        public readonly int ManipulatedVariablesDim;  // 控制变量维度 (m)
        public readonly int OutputVariablesDim;       // 输出变量维度 (p)
        public readonly int StateVariablesDim;        // 状态变量维度 (n)

        public MpcSys(Matrix<double> a, Matrix<double> b, Matrix<double> c, Matrix<double> d, double ts)
        {
            if (ts <= 0) throw new ArgumentOutOfRangeException(nameof(ts), "采样周期Ts必须为正数");

            A = a ?? throw new ArgumentNullException(nameof(a), "状态空间矩阵A不能为空");
            B = b ?? throw new ArgumentNullException(nameof(b), "状态空间矩阵B不能为空");
            C = c ?? throw new ArgumentNullException(nameof(c), "状态空间矩阵C不能为空");
            D = d;
            Ts = ts;

            _ = DimCheck();
            StateVariablesDim = A.RowCount;
            ManipulatedVariablesDim = B.ColumnCount;
            OutputVariablesDim = C.RowCount;
        }

        public MpcSys(Matrix<double> a, Matrix<double> b, Matrix<double> c, double ts)
            : this(a, b, c, null, ts)
        {
        }

        public bool SysUpdate(Matrix<double> a, Matrix<double> b, Matrix<double> c,
            Matrix<double> d)
        {
            // 维度检测
            if (a == null || b == null || c == null)
                throw new ArgumentNullException("状态空间矩阵A、B、C不能为空");
            // 矩阵更新
            A = a;
            B = b;
            C = c;
            D = d;
            return true;
        }

        public bool SysUpdate(Matrix<double> a, Matrix<double> b, Matrix<double> c, bool isToInc = true)
        {
            // 维度检测
            if (a == null || b == null || c == null)
                throw new ArgumentNullException("状态空间矩阵A、B、C不能为空");

            if (isToInc)
            {
                var temp_sys = ToIncSys(a, b, c, Ts);
                A = temp_sys.A;
                B = temp_sys.B;
                C = temp_sys.C;
                D = temp_sys.D; // 增量系统的D矩阵可以为null
            }
            else
            {
                // 矩阵更新
                A = a;
                B = b;
                C = c;
                D = null; // 增量系统不需要D矩阵
            }
            return true;
        }

        public bool DimCheck()
        {
            int n = A.RowCount;
            int m = B.ColumnCount;
            int p = C.RowCount;

            if (A.RowCount != A.ColumnCount)
                throw new ArgumentException($"A矩阵必须为方阵，当前维度：{A.RowCount}×{A.ColumnCount}");

            ValidateDimension(B, n, m, "B");
            ValidateDimension(C, p, n, "C");
            if (D != null) ValidateDimension(D, p, m, "D");

            return true;
        }

        private void ValidateDimension(Matrix<double> matrix, int expectedRows, int expectedCols, string matrixName)
        {
            if (matrix == null)
                throw new ArgumentNullException(matrixName, $"{matrixName}矩阵不能为空");
            if (matrix.RowCount != expectedRows || matrix.ColumnCount != expectedCols)
                throw new ArgumentException(
                    $"{matrixName}矩阵维度错误，期望 {expectedRows}×{expectedCols}，实际：{matrix.RowCount}×{matrix.ColumnCount}"
                );
        }

        public MpcSys ToIncSys()
        {
            int n = StateVariablesDim;
            int m = ManipulatedVariablesDim;
            int p = OutputVariablesDim;

            if (n <= 0 || m <= 0 || p <= 0)
                throw new InvalidOperationException("系统维度参数无效，无法进行增量系统转换。请检查原始系统矩阵。");

            try
            {
                // 增量系统转换：将状态空间模型转换为增量形式 A_aug = [A B; 0 I]
                var A_aug = Matrix<double>.Build.Dense(n + m, n + m, 0.0);
                A_aug.SetSubMatrix(0, n, 0, n, A);
                A_aug.SetSubMatrix(0, n, n, m, B);
                A_aug.SetSubMatrix(n, m, n, m, Matrix<double>.Build.DenseIdentity(m, m));

                // 增量输入矩阵 B_aug = [B; I]
                var B_aug = Matrix<double>.Build.Dense(n + m, m, 0.0);
                B_aug.SetSubMatrix(0, n, 0, m, B);
                B_aug.SetSubMatrix(n, m, 0, m, Matrix<double>.Build.DenseIdentity(m, m));

                // 增量输出矩阵 C_aug = [C D; 0 0]
                var D_temp = D ?? Matrix<double>.Build.Dense(p, m, 0.0);
                var C_aug = Matrix<double>.Build.Dense(p, n + m, 0.0);
                C_aug.SetSubMatrix(0, p, 0, n, C);
                C_aug.SetSubMatrix(0, p, n, m, D_temp);

                return new MpcSys(
                    a: A_aug,
                    b: B_aug,
                    c: C_aug,
                    d: Matrix<double>.Build.Dense(p, m, 0.0),
                    ts: Ts
                );
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"增量系统转换失败: {ex.Message}", ex);
            }
        }

        public MpcSys ToIncSys(Matrix<double> a, Matrix<double> b, Matrix<double> c, double ts)
        {
            int n = StateVariablesDim;
            int m = ManipulatedVariablesDim;
            int p = OutputVariablesDim;

            if (n <= 0 || m <= 0 || p <= 0)
                throw new InvalidOperationException("系统维度参数无效，无法进行增量系统转换。请检查原始系统矩阵。");

            try
            {
                // 增量系统转换：将状态空间模型转换为增量形式 A_aug = [A B; 0 I]
                var A_aug = Matrix<double>.Build.Dense(n + m, n + m, 0.0);
                A_aug.SetSubMatrix(0, n, 0, n, a);
                A_aug.SetSubMatrix(0, n, n, m, b);
                A_aug.SetSubMatrix(n, m, n, m, Matrix<double>.Build.DenseIdentity(m, m));

                // 增量输入矩阵 B_aug = [B; I]
                var B_aug = Matrix<double>.Build.Dense(n + m, m, 0.0);
                B_aug.SetSubMatrix(0, n, 0, m, b);
                B_aug.SetSubMatrix(n, m, 0, m, Matrix<double>.Build.DenseIdentity(m, m));

                // 增量输出矩阵 C_aug = [C D; 0 0]
                var D_temp = Matrix<double>.Build.Dense(p, m, 0.0);
                var C_aug = Matrix<double>.Build.Dense(p, n + m, 0.0);
                C_aug.SetSubMatrix(0, p, 0, n, c);
                C_aug.SetSubMatrix(0, p, n, m, D_temp);

                return new MpcSys(
                    a: A_aug,
                    b: B_aug,
                    c: C_aug,
                    d: null,
                    ts: ts
                );
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"增量系统转换失败: {ex.Message}", ex);
            }
        }
    }
}
