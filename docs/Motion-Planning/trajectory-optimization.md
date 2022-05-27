### The Trajectory Optimization Problem

 single-phase continuous-time trajectory optimization problems
$$
\text{system dynamics:\qquad} \dot{x}(t) = f(t, x(t), u(t))\\
\text{path constraint:\qquad} h(t, x(t), u(t)) ≤ 0\\
\text{boundary constraint:\qquad} g(t_0, t_F , x(t_0), x(t_F )≤ 0\\
\text{path bound on state:\qquad} xlow ≤ x(t) ≤ xupp\\
\text{path bound on control:\qquad} ulow ≤ u(t) ≤ uupp\\
\text{bounds on initial and final time:\qquad} tlow ≤ t0 < tF ≤ tupp\\
\text{bound on initial state:\qquad} x0,low ≤ x(t0) ≤ x0,upp\\
\text{bound on final state:\qquad} xF,low ≤ x(tF ) ≤ xF,upp
$$
两个配置点之间的状态变化等于系统动力学的积分，而积分在被积函数不确定时，用始末值计算梯形面积代替定积分：
$$
xk+1 − xk ≈ 1 2(hk)(νk+1 + νk)
$$


### Unconstrained case: BVP

$$
min \int_0^T v(t)^\top W v(t) dt \\
s.t. z^{(s)}(t)=v(t)\\
z{[s-1]}(t_0)=z_0\\
z{[s-1]}(t_M)=z_f
$$



*最优条件：s阶导数作为优化量: 轨迹为2s-1 阶多项式，eg:

s=3 minimize jerk (angular velocity, thrust): 5 degree polynomial

s=4 minimize snap(angular acc, torque): 7 degree





生成轨迹可行性检验：

- 离散时间法: 高分辨率较慢，低分辨率可能检测不到（间隔太大）

- Recursive Bound Checker:  仅限 degree 5， 分辨率相关

- extreme Value Checker：数值解（慢），degree5

  更好方法：（针对多元多项式）

$$
满足约束 \Leftrightarrow \mathscr{G}(t)<0, t\in[0,T]\\
\text{Check t=0 and t=T, if satisfied,} \Leftrightarrow \mathscr{G}(t)=0 \text{ has roots in (0,T)}
$$

Sturm Theory，检查多项式根的存在性



---

Position - Velocity - Acceleration - Jerk - Snap - Crackle - Pop

---



### Unconstrained case: BIVP (中间点)

BVP中的起始点所有量都必须指定，BIVP只指定部分中间点条件（给定节点p，自动优化v,a,j）更顺滑。

























