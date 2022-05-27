# Ceres 使用入门

## 1. 问题定义
$$
\begin{align}
\min_x \frac{1}{2}\sum_i &\rho_i (\|f_i(x_{i1},\dots,x_{ik})\|) \\\\  
\text{s.t. } l_i&\leq x_j\leq u_i
\end{align}
$$

## 2. 定义Costfunction
Given[x1,x2,...,xk] , CostFunction is responsible for computing the vector f(x1,x2,...,xk) and the Jacobian matrices:
$$
J_i=D_if(x_1,...,x_k)
$$

$$  
J=\begin{bmatrix}
\frac{\partial f_{1}}{\partial x_1}&\dots&\frac{\partial f_{1}}{\partial x_c} \\\\
\vdots&\vdots&\vdots \\\\
\frac{\partial f_{r}}{\partial x_1}&\dots&\frac{\partial f_{r}}{\partial x_c}
\end{bmatrix}_{num\_residuals \times parameter\_block\_sizes\_[i]}
$$

$$
\text{jacobians}[i][r * parameter\_block\_sizes\_[i] + c] =\frac{\partial \text{ residual[r]}}{\partial \text{ parameters}[i][c]}
$$
   
### 2.1 自动求导
  ``` C++ title="自动求导"
  // 定义自动求导的残差伪函数
  template <typename _T1> struct MyCostFunctor
  {
    //定义变量
    MyCostFunctor(var1,var2...) : var1_(var1), var2_(var2),... {}
    template <typename _T2>
    //重载括号并定义残差
    bool operator() (
      //输入：待优化的参数
      const _T2* const param1,const _T2* const param2,...,
      //输入：残差
      _T2* residuals) const {
      residuals = f(param1,param2...);
      return true;
    }
    //用工厂函数创建Costfunction
    static ceres::CostFunction* Create (const vars)
    {
      return (
      new AutoDiffCostFunction<MyCostFunctor, n_res,n_param1,n_param2,...>(
        new MyCostFunctor(var1,var2...))      ^       ^       ^
                                              |       |       |
                  Dimension of residual ------+       |       |
                  Dimension of param1 ----------------+       |
                  Dimension of param2 ------------------------+
    }
  }
  ```

  ``` C++ title="工厂模式函数构造costfunction"
  ceres::CostFunction* cost_function = MyCostFunctor<_T>::Create ( 
  var1,var2...);
  ```

### 2.2 解析求导
``` C++  title="解析求导模板"
class AnalyticCostFunction : public ceres::SizedCostFunction<n_res,n_param1,n_param2,...> {
  public:
      AnalyticCostFunction(var1, var2,...): var1_(var1), var2_(var2), ... {}

  virtual bool Evaluate(double const* const* parameters, 
                        double* residuals,  
                        double** jacobians) const  
      { 
        //变量
        var1=parameters[0];
        var2=parameters[1];
        ...
        //计算残差
        residuals = f(var1, var2,...);
        //计算雅可比
        jacobians = j(var1, var2,...);
        return true;
      }
  //储存变量
  protected:
    var1, var2,...;
  };
```
!!! info ""
    `parameters`是`CostFunction::parameter_block_sizes_.size()`大小的array.  
    `parameters[i]` 是`parameter_block_sizes_[i]`大小的包含第i个parameter block需要参数的array.  
    `parameters` is never `nullptr`.  
!!! info ""
    `residuals` is an array of size `num_residuals_`.  
    `residuals` is never `nullptr`.  
!!! info ""
    `jacobians` is an array of arrays of size `CostFunction::parameter_block_sizes_.size()`.  
    If `jacobians` is `nullptr`, the user is only expected to compute the residuals.  
    `jacobians[i]` is a row-major array of size `num_residuals x parameter_block_sizes_[i]`  
    If `jacobians[i]` is **not** `nullptr`, the user is required to compute the Jacobian of the residual vector with respect to `parameters[i]` and store it in this array  
    If `jacobians[i]` is `nullptr`, then this computation can be skipped. This is the case when the corresponding parameter block is marked constant.
!!! warning ""
    `jacobians` 通常定义为双重指针 `double **`, 而计算出的雅可比通常为`Eigen::MatrixXd`,需要用Map函数映射，如：
    ```
    Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
    ```
```C++ title="定义costfunction"
ceres::CostFunction *cost_function = new AnalyticCostFunction(var1, var2,...);
```

## 3. 构造优化问题
```C++
//构造优化问题
ceres::Problem problem;
ceres::CostFunction* cost_function = ... // (Auto/Analytic cost function)
//添加参数块
problem.AddParameterBlock(*var,size)
...
//添加残差块
problem.AddResidualBlock ( 
  cost_function,           //cost fuction
  loss,                    //loss: eg.squared loss,HuberLoss
  parameter_blocks,...
  ); 
```

!!! Note "Problem::AddParameterBlock( )"
    用户在调用AddResidualBlock( )时其实已经隐式地向Problem传递了参数模块，但在一些情况下，需要用户显示地向Problem传入参数模块（通常出现在需要对优化参数进行重新参数化的情况）。
    ``` C++
    void Problem::AddParameterBlock(double *values, int size)
    void Problem::AddParameterBlock(double *values, int size, LocalParameterization *local_parameterization)
    ```
    
```C++ title="以LOAM中点云匹配求导作为例子"
double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);
...
problem.AddParameterBlock(parameters, 4, q_parameterization);
problem.AddParameterBlock(parameters + 4, 3);
```
!!! Note "Problem::AddResidualBlock( )"
    向Problem类传递残差模块的信息
    ``` C++
    ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
                        LossFunction *loss_function, 
                        const vector<double *> parameter_blocks)
                        
    ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
                          LossFunction *loss_function,
                          double *x0, double *x1, ...)
    ```
```C++ title="以LOAM中点云匹配求导作为例子"
ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
```

## 4. 求解

``` C++
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_QR;
...
//求解
ceres::Solver::Summary summary;
ceres::Solve (options, &problem, &summary); 
```