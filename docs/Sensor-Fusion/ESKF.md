
## 1. ESKF 简介
原状态变量称为**名义状态变量**（nominal state）即不考虑噪声的状态。

真实的状态为**真值状态**(true state), 需要考虑噪声。

  ESKF里的状态变量称为**误差状态变量**（error state), 他们的关系为`误差状态变量= 真值状态-名义状态变量`, 误差状态变量

ESKF predicts a Gaussian estimate of the error-state

**ESKF 优点**：

- error state中旋转变量可以用最小表示，即旋转向量表示，没有多余的变量，可以避免冗余变量带来的singularity risk
- error-state system 通常都在原点附近，可以避免万向节锁，奇异值等问题，保证linearization每时每刻都是可行的
- error state 通常都很小，因此二阶量可以忽略，雅可比的计算更快速
- error state dynamics比较慢，因为大信号在名义动力学中被积分了，剩下的error是小信号，因此ESKF可以工作在较低的频率下

ESKF流程：

> In parallel with integration of the nominal state, the ESKF predicts a Gaussian estimate of the error-state. It only predicts, because by now no other measurement is available to correct these estimates. The filter correction is performed at the arrival of information other than IMU (e.g. GPS, vision, etc.), which is able to render the errors observable and which happens generally at a much lower rate than the integration phase. This correction provides a posterior Gaussian estimate of the error-state. After this, the error-state’s mean is injected into the nominal-state, then reset to zero. The error-state’s covariances matrix is conveniently updated to reflect this reset. The system goes on like this forever


## 2. 真值状态动力学方程
**真值状态动力学方程**，这些状态是在世界坐标系下的。  

$$
\begin{align}
&\text{Position:}&\dot{p_t}&=v_t\\
&\text{Velocity:}&\dot{v_t}&=a_t=R_t(a_m-bias_{at}-\eta_a)+g_t\\
&\text{Quaternion:}&\dot{q_t}&=\frac{1}{2}q_t \otimes \omega_t=\frac{1}{2}q_t \otimes (\omega_m-bias_{gt}-\eta_g )\\
&\text{Accelerometer bias:}&\dot{bias}_{at}&=\eta_{ba}(noise)\\
&\text{Gyrometer bias:}&\dot{bias}_{gt}&=\eta_{bg}(noise)\\
&\text{Gravity vector:}&\dot{g_t}&=0\\
\end{align}
$$

其中，加速度at和角速度wt是由IMU测量值计算出来的：  

$$
\begin{align}
a_m&=R_t^\top(a_t-g_t)+bias_a+\eta_a &\Rightarrow a_t&=R_t(a_m-bias_a-\eta_a)+g_t\\
\omega_m&=\omega_t+bias_g+\eta_g&\Rightarrow \omega_t&=\omega_m-bias_g-\eta_g 
\end{align}
$$

## 3.名义状态动力学方程
**名义状态动力学方程**，这些状态是在世界坐标系下的。  

$$
\begin{align}
&\text{Position:}&\dot{p}&=v\\
&\text{Velocity:}&\dot{v}&=R(a_m-bias_a)+g\\
&\text{Quaternion:}&\dot{q}&=\frac{1}{2}q \otimes (\omega_m-bias_g)\\
&\text{Accelerometer bias:}&\dot{bias}_{a}&=0\\
&\text{Gyrometer bias:}&\dot{bias}_{g}&=0\\
&\text{Gravity vector:}&\dot{g_t}&=0\\
\end{align}
$$

## 4.误差状态动力学方程
**误差状态动力学方程** (error-state kinematics)  

$$
\begin{align}
&\text{Position:}& \delta \dot{p}&=\delta v\\
&\text{Velocity:}& \delta \dot{v}&=R[a_m-bias_a]_{times}\delta \theta -R \delta a_b +\delta g -R\eta_a\\
&\text{Quaternion:}& \delta \dot{\theta}&=-[\omega_m-bias_g]_{\times}\delta \theta - \delta bias_g - \eta_g\\
&\text{Accelerometer bias:}& \delta \dot{bias}_{a}&=\eta_{ba}\\
&\text{Gyrometer bias:}& \delta \dot{bias}_{g}&=\eta_{bg}\\
&\text{Gravity vector:}& \delta \dot{g}&=0\\
\end{align}
$$

### 速度误差(Linear velocity error) 动力学推导:  

$$
\begin{align}
R_t& = R(I+[\delta \theta]_{\times})\\
bias_{at}&=bias_a+\delta bias_a\\
g_{t}&=g+\delta g\\
a_B& \doteq a_m-bias_a\\
\delta a_B&\doteq -\delta bias_a-\eta_{ba}\\
\Rightarrow \dot{v}&=R a_B +g\\
\dot{v}+\dot{\delta v}=\dot{v_t}&=R_t(a_m-bias_{at}-\eta_a)+g_t\\
&=R(I+[\delta \theta]_{\times})(\underbrace{a_m-bias_a}_{a_B}+\underbrace{-\delta bias_a-\eta_a}_{\delta a_B})+g_t\\
&=R(I+[\delta \theta]_{\times})(a_B+\delta a_B)+g+\delta g\\
\Rightarrow \cancel{R a_B} +\cancel{g} +\dot{\delta v}=\dot{v_t}&=\cancel{R a_B}+R[\delta \theta]_{\times}a_B+R\delta a_B+R[\delta \theta]_{\times}\delta a_B+\cancel{g}+\delta g\\
\Rightarrow \dot{\delta v}&=R[\delta \theta]_{\times}a_B+R\delta a_B+\cancel{R[\delta \theta]_{\times}\delta a_B}(二阶小量)+\delta g\\
&=-R[a_B]_{\times}\delta \theta+R\delta a_B+\delta g\\
&=-R[a_m-bias_a]_{\times}\delta \theta+R(-\delta bias_a-\eta_{ba})+\delta g\\
\dot{\delta v}&=-R[a_m-bias_a]_{\times}\delta \theta-R\delta bias_a+\delta g-R\eta_{ba}\\
\end{align}
$$

如果三轴加速度计是同型号的，可以假设其加速度计噪声是独立同分布且各项同性的，即其噪声的均值和协方差与旋转无关。

即$\eta_{ba}=R \eta_{ba}$，则可以进一步简化：

$$
\dot{\delta v}=-R[a_m-bias_a]_{\times}\delta \theta-R\delta bias_a+\delta g-\underline{\eta_{ba}}\\
$$
### 方向误差(orientation error) 动力学推导:  

$$
\begin{align}
(定义)q_t&=q \otimes \delta q\\
\dot{q_t}&=\frac{1}{2}q_t \otimes \omega_t\\
\dot{q}&=\frac{1}{2}q \otimes \omega\\
bias_{gt}&=bias_g+\delta bias_g\\
(定义)\omega_t &= \omega + \delta \omega \\
(计算)\omega_t &=\omega_m-bias_{gt}-\eta_g \\
&=\underbrace{\omega_m-bias_g}_{\omega}+\underbrace{-\delta bias_g-\eta_g}_{\delta\omega}\\
\frac{1}{2}q_t \otimes \omega_t=\dot{q_t}&=(\dot{q \otimes \delta q})\\
\Rightarrow\frac{1}{2}q \otimes \delta q \otimes \omega_t=\dot{q_t}&=\dot{q} \otimes \delta q+q \otimes \dot{\delta q}\\
\Rightarrow\frac{1}{2} \cancel{q \otimes} \delta q \otimes \omega_t&=\frac{1}{2} \cancel{q \otimes} \omega \otimes \delta q+\cancel{q \otimes} \dot{\delta q}\\
\Rightarrow 2\dot{\delta q} &=\delta q \otimes \omega_t-\omega \otimes \delta q
\end{align}
$$

!!! note "四元数乘法"  
  $[\mathbf{p}]_L=p_w\mathbf{I}+\begin{bmatrix}0&-\mathbf{p}_v^\top\\\mathbf{p}_v&[\mathbf{p}_v]_{\times}\end{bmatrix}$  

  $[\mathbf{q}]_R=q_w\mathbf{I}+\begin{bmatrix}0&-\mathbf{q}_v^\top\\\mathbf{q}_v&-[\mathbf{q}_v]_{\times}\end{bmatrix}$  

将向量(纯虚四元数)的四元数乘法带入打开:  

$$
\begin{align}
\Rightarrow 2\dot{\delta q} &=[\omega_t]_{R}\delta q  -[\omega]_L\delta q\\
&=\begin{bmatrix}
0&-\omega_t^\top\\
\omega_t&-[\omega_t]_{\times}
\end{bmatrix}\delta q
-
\begin{bmatrix}
0&-\omega^\top\\
\omega & [\omega]_{\times}
\end{bmatrix}\delta q\\
&=\begin{bmatrix}
0&-(\omega_t-\omega)^\top\\
\omega_t-\omega&-[\omega_t+\omega]_{\times}
\end{bmatrix}\begin{bmatrix}1 \\ \frac{1}{2}\delta\theta\end{bmatrix}\\
\Rightarrow 2\begin{bmatrix}0 \\ \frac{1}{2}\delta\dot{\theta}\end{bmatrix}&=\begin{bmatrix}
0&-\delta\omega^\top\\
\delta\omega &-[\delta\omega+2\omega]_{\times}
\end{bmatrix}\begin{bmatrix}1 \\ \frac{1}{2}\delta\theta\end{bmatrix}\\
\Rightarrow &
\begin{cases}
        0 = -\delta\omega^\top \delta\theta(二阶小量忽略)\\
        \delta\dot{\theta} = \delta\omega -[\delta\omega+2\omega]_{\times}\frac{1}{2}\delta\theta
\end{cases}\\
\Rightarrow \delta\dot{\theta} &= \delta\omega 
\cancel{-\frac{1}{2}[\delta\omega]_{\times}\delta\theta}
-[\omega]_{\times}\delta\theta\\
\Rightarrow \delta\dot{\theta} &= \delta\omega-[\omega]_{\times}\delta\theta\\
&=-[\omega_m-bias_g]_{\times}\delta\theta-\delta bias_g-\eta_g\\
\end{align}
$$

## 5.离散误差状态动力学方程
**误差状态动力学方程** (error-state kinematics)  

$$
\begin{align}
\delta \dot{p}& \leftarrow \delta \dot{p} +\delta v \Delta t\\
\delta \dot{v}&\leftarrow \dot{v}+(R[a_m-bias_a]_{times}\delta \theta -R \delta a_b +\delta g)\Delta t + \underbrace{-\eta_a\Delta t}_{v_i}\\
\delta \dot{\theta}&\leftarrow \delta \dot{\theta} -([\omega_m-bias_g]_{\times}\delta \theta - \delta bias_g)\Delta t + \underbrace{ - \eta_g \Delta t}_{\theta_i}\\\
\delta \dot{bias}_{a}&\leftarrow \delta \dot{bias}_{a} + \underbrace{\eta_{ba}\Delta t}_{a_i}\\\
\delta \dot{bias}_{g}&\leftarrow \delta \dot{bias}_{g} + \underbrace{\eta_{bg}\Delta t}_{\omega_i}\\\
\delta \dot{g}&\leftarrow \dot{g}\\
\end{align}
$$

将误差动力学系统重新写成状态方程的形式：
状态:$x=\begin{bmatrix}
p\\  
v\\
q\\
bias_a\\
bias_g\\
g
\end{bmatrix}$一阶导数:$\delta x=\begin{bmatrix} 
\delta p\\  
\delta v\\  
\delta \theta\\  
\delta bias_a\\  
\delta bias_g\\  
\delta g
\end{bmatrix}$输入（测量值）:$u_m=\begin{bmatrix}
a_m\\  
\omega_m\\
\end{bmatrix}$噪声:$i=\begin{bmatrix}
v_i\\  
\theta_i\\
a_i\\
\omega_i
\end{bmatrix}$
得到:
$\delta x \leftarrow F_x(x,u_m)\cdot \delta x+F_i \cdot i$
