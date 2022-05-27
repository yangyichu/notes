!!! note "关于Jacobian"
    Jacobian(f,v) computes the Jacobian matrix of f(scalar function) with respect to v.  
    The (i,j) element of the result is $\frac{\partial f(i)}{\partial v(j)}$  
    Jacobian matrix 与v的方向无关（列向量 or 行向量） 
    $$
        Jacobian([f_1,f_2,f_3],[v_1,v_2,v_3])=Jacobian([f_1,f_2,f_3],[v_1,v_2,v_3]^\top)=\begin{bmatrix}
        \frac{\partial f_1}{\partial v_1}&\frac{\partial f_1}{\partial v_2}&\frac{\partial f_1}{\partial v_3} \\\\
        \frac{\partial f_2}{\partial v_1}&\frac{\partial f_2}{\partial v_2}&\frac{\partial f_2}{\partial v_3} \\\\
        \frac{\partial f_3}{\partial v_1}&\frac{\partial f_3}{\partial v_2}&\frac{\partial f_3}{\partial v_3} 
        \end{bmatrix}
    $$

## 线特征
残差：  

$$
d_{\epsilon}=
\frac{\|(\tilde{p}_i-p_a)\times(\tilde{p}_i-p_b)\|}{\|p_a-p_b\|}
$$

!!! note "Jacobian of |x|"
    $$
        Jacobian(\|x\|,x)=Jacobian(\|x\|,x^\top)=
        \frac{1}{2}\frac{(2x_1,2x_2,2x_3)}{\sqrt{x^2_1+x^2_2+x^2_3}}=
        \frac{(x_1,x_2,x_3)}{\sqrt{x^2_1+x^2_2+x^2_3}}
    $$

雅可比：

$$
\begin{align}
    J_1
    &=\frac{1}{\|p_a-p_b\|}\frac{\partial \|X\|}{\partial X}\frac{\partial X}{\partial p}\\
    &=\frac{1}{\|p_a-p_b\|}\frac{((\tilde{p}_i-p_a)\times(\tilde{p}_i-p_b))^\top}{\|(\tilde{p}_i-p_a)\times(\tilde{p}_i-p_b)\|}(p_b-p_a)^{\wedge}\\
    J_2
    &=[\frac{\partial \tilde{p}_i}{\partial \phi},\frac{\partial \tilde{p}_i}{\partial t}]\\
    \frac{\partial \tilde{p}_i}{\partial \delta\phi} &=-(Rp_i)^{\wedge}\\
    \frac{\partial \tilde{p}_i}{\partial t}&=\frac{\partial (R\tilde{p}_i+t)}{\partial t}=\text{I}
\end{align}
$$

## 面特征
残差：  

$$
d_{\epsilon}=
\|\left(\tilde{p}_i-p_{j}\right) \cdot \frac{\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)}{\|\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)\|}\|
$$

雅可比：

$$
    \begin{align}
    J_1
    &=\frac{d^\top}{\|d\|}\frac{(\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right))^\top}{\|\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)\|}\\
    J_2
    &=[\frac{\partial \tilde{p}_i}{\partial \phi},\frac{\partial \tilde{p}_i}{\partial t}]\\
    \frac{\partial \tilde{p}_i}{\partial \delta\phi}&=-(Rp_i)^{\wedge}\\
    \frac{\partial \tilde{p}_i}{\partial t}&=\frac{\partial (R\tilde{p}_i+t)}{\partial t}=\text{I}
    \end{align}
$$



!!! note ""
    $$
        (a \times b )^\top = a^\top \times b^\top 
    $$


```C++
class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 4, 3>
{ 
public:
    double s;
    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    EdgeAnalyticCostFunction(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                const Eigen::Vector3d last_point_b_, const double s_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

    virtual bool Evaluate(double const *const *parameters,
                            double *residuals,
                            double **jacobians) const 
    {
        Eigen::Map<const  Eigen::Quaterniond>   q_last_curr(parameters[0]);               
        Eigen::Map<const  Eigen::Vector3d>      t_last_curr(parameters[1]);
        Eigen::Vector3d  lp ;  
        Eigen::Vector3d  lp_r ;
        lp_r =  q_last_curr * curr_point;
        lp   =  q_last_curr * curr_point  + t_last_curr;
        Eigen::Vector3d  v1 =  (lp - last_point_a);
        Eigen::Vector3d  v2 =  (lp - last_point_b);
        Eigen::Vector3d  v3 =  last_point_a  - last_point_b;
    
        residuals[0] = (v1.cross(v2)).norm() / v3.norm(); 
        Eigen::Vector3d dXnorm_dX = v1.cross(v2)/(v1.cross(v2)).norm();
        Eigen::Matrix3d dX_dp = -skew(v3);
        Eigen::RowVector3d J1 = 1/ v3.norm()*dXnorm_dX.transpose()*dX_dp;
        if (jacobians !=  NULL)
        {
            if (jacobians[0]  !=  NULL)
            {
                Eigen::Matrix3d    J2= -skew(lp_r); //3x3
                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
                J_so3_r.setZero();
                J_so3_r.block<1,3>(0,1)  =  J1*J2; //1x3

                Eigen::Map<Eigen::Matrix<double,  1,  3, Eigen::RowMajor>> J_so3_t(jacobians[1]);
                J_so3_t.setZero();
                J_so3_t.block<1,3>(0,0)  = J1; //1x3
            }
        }
        return true;
    }
};
```

```C++
class PlaneAnalyticCostFunction : public ceres::SizedCostFunction<1, 4, 3>
{
public:
    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;

    PlaneAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                                Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
        : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), last_point_m(last_point_m_), s(s_) {}

    virtual bool Evaluate(double const *const *parameters,
                            double *residuals,
                            double **jacobians) const
    { 
        Eigen::Map<const Eigen::Quaterniond>  q_last_curr(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);

        Eigen::Vector3d  lp_r = q_last_curr *  curr_point ; 
        Eigen::Vector3d  lp = q_last_curr *  curr_point  +  t_last_curr;  
        
        Eigen::Vector3d  ljm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
        Eigen::Vector3d  plane_norm = ljm /ljm.norm(); //3x1
        Eigen::Vector3d  v_ij = lp - last_point_j;
        double  d = plane_norm.transpose()*v_ij;    //  1x1

        residuals[0] =  std::abs(d);//1*1

        Eigen::RowVector3d J1 = d* plane_norm.transpose()/residuals[0];
        if(jacobians != NULL)
        {
            if(jacobians[0] != NULL)
            {
                Eigen::Matrix3d    J2(-skew(lp_r)); //3x3
                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>>  J_so3_r(jacobians[0]);
                J_so3_r.setZero();
                J_so3_r.block<1,3>(0,1) =J1*J2;

                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>  J_so3_t(jacobians[1]);
                J_so3_t.setZero();
                J_so3_t.block<1,3>(0,0)  = J1;    
            }
        }
        return true;
    }
};
```