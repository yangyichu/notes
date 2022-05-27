## Some Problem
  `PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet, nodelet::Nodelet);` 
  改为`PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);`

## 流程
```C++
 _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);

  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);

  _traj_pub =
      nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);
```

!!! info "vector 截取"
  ```C++
  vector<int>::const_iterator First = Arrs.begin();
  vector<int>::const_iterator Second = Arrs.begin();
  //初始化截取
  vector<int> Arrs2(First, Second);
  //assign()函数截取
  vector<int> Arr2;
  Arr2.assign(First,Second); 
  ```

```C++
function DouglasPeucker(PointList[], epsilon)
    // Find the point with the maximum distance
    dmax = 0
    index = 0
    end = length(PointList)
    for i = 2 to (end - 1) {
        d = perpendicularDistance(PointList[i], Line(PointList[1], PointList[end])) 
        if (d > dmax) {
            index = i
            dmax = d
        }
    }
    
    ResultList[] = empty;
    
    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        // Recursive call
        recResults1[] = DouglasPeucker(PointList[1...index], epsilon)
        recResults2[] = DouglasPeucker(PointList[index...end], epsilon)

        // Build the result list
        ResultList[] = {recResults1[1...length(recResults1) - 1], recResults2[1...length(recResults2)]}
    } else {
        ResultList[] = {PointList[1], PointList[end]}
    }
    // Return the result
    return ResultList[]
end
```

## 时间分配
使用梯形速度方案（trapezoidal velocity profile）
给定最大速度，最大加速度和距离，可以解得最小时间。
达到最大速度并减速到0所需距离： 
$s_{min}=2 \times \frac{1}{2}a_{max}\dfrac{v^2_{max}}{a^2_{max}}=\dfrac{v^2_{max}}{a_{max}}$
1. 两点距离不够达到最大速度前并减速到0:  
$a_{max}t^2_{half}=s\Rightarrow t_{half}=\sqrt{\dfrac{s}{a_{max}}}\Rightarrow t = 2 t_{half}$
2. 两点距离足够达到最大速度前并减速到0:  
$t=2\dfrac{v_{max}}{a_{max}}+\dfrac{s-s_{min}}{v_{max}}$

```C++
const double t = vel / acc;
const double d = 0.5 * acc * t * t;

if (dist < d + d){
    return 2.0 * sqrt(dist / acc);
}
else{
    return 2.0 * t + (dist - 2.0 * d) / vel;
}
```

## minimum snap 解算
给定n-1段轨迹中的n个必经点，以及n-1段的时间  
BVP求解  
state $\bm{s}=[\bm{p},\bm{v},\bm{a}]^\top$, trajectory $\bm{s}(t)=\bm{M}(t)\bm{c}$, in which $\bm{c}=[\bm{c_0},\bm{c_1},\bm{c_2},\bm{c_3},\bm{c_4},\bm{c_5}]^\top,\bm{c_i}=[c_{ix},c_{iy},c_{iz}]$  

$$
\bm{M}(t)=
\begin{bmatrix}
1 & t & t^2 & t^3 & t^4 & t^5\\
0 & 1 & 2t & 3t^2 & 4t^3 & 5t^4\\
0 & 0 & 2 & 6t & 12t^2 & 20t^3
\end{bmatrix}
$$

Given $\bm{s}_0,\bm{s}_T$,  

$$
\begin{bmatrix}
\bm{s}(0)\\
\bm{s}(T)
\end{bmatrix}_{6\times 3}=
\begin{bmatrix}
\bm{M}(0)\\
\bm{M}(T)
\end{bmatrix}_{6\times 6}
\begin{bmatrix}
\bm{c}_0
\end{bmatrix}_{6\times 3}
\Rightarrow 
\begin{bmatrix}
\bm{c}_0
\end{bmatrix}
=
\begin{bmatrix}
\bm{M}(0)\\
\bm{M}(T)
\end{bmatrix}^{-1}
\begin{bmatrix}
\bm{s}(0)\\
\bm{s}(T)
\end{bmatrix}
$$

BIVP求解  
state $\bm{s}=[\bm{p},\bm{v},\bm{a}]^\top$,intermediate state $\bm{s}_i=[\bm{p},\bm{v},\bm{a},\bm{j},\bm{s}]^\top$  
trajectory $\bm{s}(t)=\bm{M}(t)\bm{c}$, in which $\bm{c}=[\bm{c_0},\bm{c_1},\bm{c_2},\bm{c_3},\bm{c_4},\bm{c_5}]^\top$  
Boundary M,
$$
\bm{M}(t)=
\begin{bmatrix}
1 & t & t^2 & t^3 & t^4 & t^5\\
0 & 1 & 2t & 3t^2 & 4t^3 & 5t^4\\
0 & 0 & 2 & 6t & 12t^2 & 20t^3
\end{bmatrix}
$$
Intermediate M,  
$$
\bm{M}_i(t)=
\begin{bmatrix}
1 & t & t^2 & t^3 & t^4 & t^5\\
0 & 1 & 2t & 3t^2 & 4t^3 & 5t^4\\
0 & 0 & 2 & 6t & 12t^2 & 20t^3\\
0 & 0 & 0 & 6 & 24t & 60t^2\\
0 & 0 & 0 & 0 & 24 & 120t
\end{bmatrix}
$$  

Intermediate D,  

$$
\bm{D}_i(t)=
\begin{bmatrix}
1 & t & t^2 & t^3 & t^4 & t^5
\end{bmatrix}
$$
Given $\bm{s}_{start},\bm{s}_{end}$, with $n_{int}$ intermediate position landmarks $\bm{s}_i$

$$
\begin{align}
	\bm{s}_{start}&=\bm{M}(0)\bm{c}_0\\
	&\vdots\\
	\underbrace{
	\begin{bmatrix}
	    \bm{p}_i\\
	    \bm{0}
	\end{bmatrix}}_{\bm{b}_i}
	&=
	\begin{bmatrix}
	    \bm{0}&{\color{red}\bm{D}_i(0)}\\
	    -\bm{M}_i(T_i)&\bm{M}_i(0)
	\end{bmatrix}
	\begin{bmatrix}
	    \bm{c}_{i-1}\\
	    \bm{c}_i
	\end{bmatrix}=\begin{bmatrix}
	    {\color{blue}\bm{D}_i(T_{i-1})}&\bm{0}\\
	    -\bm{M}_i(T_i)&\bm{M}_i(0)
	\end{bmatrix}
	\begin{bmatrix}
	    \bm{c}_{i-1}\\
	    \bm{c}_i
	\end{bmatrix}=\begin{bmatrix}
	    \bm{E}_i&\bm{F}_i
	\end{bmatrix}
	\begin{bmatrix}
	    \bm{c}_{i-1}\\
	    \bm{c}_i
	\end{bmatrix}
	\\
	&\vdots\\
	\bm{s}_{end}&=\bm{M}(T_{n_{int}})\bm{c}_{end}\\
	\underbrace{
	\begin{bmatrix}
		\bm{c}_0\\
		\bm{c}_1\\
		\bm{c}_2\\
		\vdots\\
		\bm{c}_{n_{int}}
	\end{bmatrix}
	}_{\bm{c}}
	&=	\underbrace{
	\begin{bmatrix}
	    \bm{M}(0)&\bm{0}&\bm{0}&\dots&\bm{0}\\
	    \bm{E}_1&\bm{F}_1&\bm{0}&\dots&\bm{0}\\
	    \bm{0}&\bm{E}_2&\bm{F}_2&\dots&\bm{0}\\
	    \vdots&\vdots&\vdots&\ddots&\vdots\\
		\bm{0}&\bm{0}&\bm{0}&\dots&\bm{F}_{n_{int}}\\
		\bm{0}&\bm{0}&\bm{0}&\dots&\bm{M}(T_{n_{int}})
	\end{bmatrix}^{-1}}_{A^{-1}}
	\underbrace{
	\begin{bmatrix}
		\bm{s}_{start}\\
		\bm{b}_1\\
		\bm{b}_2\\
		\vdots\\
		\bm{b}_{n_{int}}\\
		\bm{s}_{end}
	\end{bmatrix}
	}_{\bm{b}}\\
	\bm{c}_{6N\times 3}&=\bm{A}^{-1}_{6N\times 6N}\bm{b}_{6N\times 3}\quad, N=n_{int}+1
\end{align}
$$