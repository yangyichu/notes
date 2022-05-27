## 函数导数

## x为标量

### F为函数向量：

$$
F=\begin{Bmatrix}
F_1(x)\\F_2(x)\\\vdots\\F_n(x)
\end{Bmatrix}
\qquad
\frac{\partial F}{\partial X}=\begin{Bmatrix}
\frac{\partial F_1}{\partial X}\\\frac{\partial F_2}{\partial X}\\\vdots\\\frac{\partial F_n}{\partial X}
\end{Bmatrix}
$$

### F为函数矩阵：

$$
F=\begin{Bmatrix}
f_{11}&\dots&f_{1n}\\\vdots&\vdots&\vdots\\f_{m1}&\dots&f_{mn}
\end{Bmatrix}
\qquad
\frac{\partial F}{\partial X}=\begin{Bmatrix}
\frac{\partial F_{11}}{\partial X}&\dots&\frac{\partial F_{1n}}{\partial X}\\\vdots&\vdots&\vdots\\\frac{\partial F_{m1}}{\partial X}&\dots&\frac{\partial F_{mn}}{\partial X}
\end{Bmatrix}
$$

#### 求导法则：

$$
\frac{d[F_1(x)\pm F_2(x)]}{dx}=\frac{dF_1(x)}{dx}\pm \frac{dF_2(x)}{dx}\\
\text{F为函数向量：}\frac{d[F_1^\top(x) F_2(x)]}{dx}=\frac{dF_1^\top(x)}{dx}F_2(x)+ F_1^\top(x)\frac{dF_2(x)}{dx}\\
\text{F为函数矩阵：}\frac{d[F_1(x) F_2(x)]}{dx}=\frac{dF_1(x)}{dx}F_2(x)+ F_1(x)\frac{dF_2(x)}{dx}
$$

## x为n维向量

### F标量函数

标量 *y* 对 *n* 维列向量 *x* (nx1) 求导，其结果还是一个 *n*维列向量：

标量 *y* 对 *n* 维行向量 *x* (1xn) 求导，其结果还是一个 *n*维行向量

**形状规则：**标量 *y* 对向量 *x* 的每个元素求导，然后将各个求导结果按向量 *x* 的形状排列。

### F为函数向量：

$$
F=\begin{Bmatrix}
F_1(x)\\F_2(x)\\\vdots\\F_m(x)
\end{Bmatrix}
\qquad
x=\begin{Bmatrix}
x_1\\x_2\\\vdots\\x_n
\end{Bmatrix}
\qquad
\frac{\partial F}{\partial X}=
\begin{Bmatrix}
\frac{\partial F}{\partial x_1}\\\frac{\partial F}{\partial x_2}\\\vdots\\\frac{\partial F}{\partial x_n}
\end{Bmatrix}=
\begin{Bmatrix}
\frac{\partial F_{1}}{\partial x_1}&\dots&\frac{\partial F_{1}}{\partial x_1}\\\vdots&\vdots&\vdots\\\frac{\partial F_{m}}{\partial x_n}&\dots&\frac{\partial F_{m}}{\partial x_n}
\end{Bmatrix}
$$

同时, 有:
$$
\frac{\partial{F}}{\partial{X}}=\frac{\partial{F}}{\partial{X}^\top}\\
\frac{\partial{F^\top}}{\partial{X}}=
\begin{Bmatrix}
\frac{\partial F^\top}{\partial x_1}&\frac{\partial F^\top}{\partial x_2}&\dots&\frac{\partial F^\top}{\partial x_n}
\end{Bmatrix}^\top=
\begin{Bmatrix}
\frac{\partial F_{1}}{\partial x_1}&\dots&\frac{\partial F_{m}}{\partial x_1}\\\vdots&\vdots&\vdots\\\frac{\partial F_{1}}{\partial x_n}&\dots&\frac{\partial F_{m}}{\partial x_n}
\end{Bmatrix}=[\frac{\partial{F}}{\partial{X^\top}}]^\top
$$

  1）向量 *y* 的每个元素是标量，先做 *y* 的每个元素对向量 *x*求导，按照标量对向量的求导规则进行。

  2）第一步做好后，将求导结果按 *y* 的形状排列， 即ny x nx。

### 链式法则

向量对向量的链式法则与普通标量对标量的相同：
$$
(\frac{\partial \mathbf{z}_{p\times1}}{\partial \mathbf{x}_{m\times1}})_{p\times m}=(\frac{\partial \mathbf{z}}{\partial \mathbf{y}})_{p\times n}(\frac{\partial \mathbf{y}}{\partial \mathbf{x}})_{n\times m}
$$
标量对向量的链式法则需要取一个转置：
$$
(\frac{\partial z_{1\times1}}{\partial \mathbf{x}_{m\times1}})_{m\times 1}=(\frac{\partial \mathbf{y}}{\partial \mathbf{x}}_{n\times m})^\top_{m\times n}(\frac{\partial z}{\partial \mathbf{y}})_{n\times 1}\\
(\frac{\partial z_{1\times1}}{\partial \mathbf{a}_{a\times1}})_{a\times 1}=(\frac{\partial \mathbf{y}}{\partial \mathbf{x}}_{y\times x} \dots \frac{\partial \mathbf{c}}{\partial \mathbf{b}}_{c\times b}\frac{\partial \mathbf{b}}{\partial \mathbf{a}}_{b\times a} )^\top_{a\times y}(\frac{\partial z}{\partial \mathbf{y}})_{y\times 1}
$$


### F为函数矩阵：

$$
F=\{{f_{ij}}\}_{nl}\\
\frac{\partial F}{\partial X}=
\begin{Bmatrix}
\frac{\partial F}{\partial x_1}\\\frac{\partial F}{\partial x_2}\\\vdots\\\frac{\partial F}{\partial x_n}
\end{Bmatrix}_{mn\times l}
$$

## X为axb维矩阵

$$
X=\begin{Bmatrix}
x_{11}&\dots&x_{1b}\\\vdots&\vdots&\vdots\\x_{a1}&\dots&x_{ab}
\end{Bmatrix}
$$

### F为标量函数

$$
\frac{\partial F}{\partial X}=
\begin{Bmatrix}
\frac{\partial F}{\partial x_{11}}&\dots&\frac{\partial F}{\partial x_{1b}}\\\vdots&\vdots&\vdots\\\frac{\partial F}{\partial x_{a1}}&\dots&\frac{\partial F}{\partial x_{ab}}
\end{Bmatrix}
$$



### F为函数向量

$$
F=\{{f_{i}}\}_{m\times 1}\\
\frac{\partial F}{\partial X}=
\begin{Bmatrix}
\frac{\partial F_1}{\partial x}\\\frac{\partial F_2}{\partial x}\\\vdots\\\frac{\partial F_m}{\partial x}
\end{Bmatrix}_{ma\times b}
$$



### F为函数矩阵

$$
F=\begin{Bmatrix}
f_{11}&\dots&f_{1n}\\\vdots&\vdots&\vdots\\f_{m1}&\dots&f_{mn}
\end{Bmatrix}\\
\frac{\partial F}{\partial X}=\begin{Bmatrix}
\frac{\partial f_{11}}{\partial X}&\dots&\frac{\partial f_{1n}}{\partial X}\\\vdots&\vdots&\vdots\\\frac{\partial f_{m1}}{\partial X}&\dots&\frac{\partial f_{mn}}{\partial X}
\end{Bmatrix}_{am\times bn}
$$

## 复合函数导数

x,y,z 为向量，t为标量，f为标量函数

- z=z(y), y=y(t)

$$
\frac{dz}{dt}=\frac{dz}{dy^\top}\frac{dy}{dt}
$$

- z=z(y), y=y(x)
  $$
  \frac{dz}{dx}=\frac{dz}{dx^\top}=\frac{dz}{dy^\top}\frac{dy}{dx^\top}\\
  \frac{dz^\top}{dx}=\frac{dy^\top}{dx}\frac{dz^\top}{dy}
  $$
  
- z=z(y,x), y=y(x)
  $$
  \frac{dz}{dx}=\frac{dz}{dx^\top}+\frac{dz}{dy^\top}\frac{dy}{dx^\top}\\
  \frac{dz^\top}{dx}=\frac{dy^\top}{dx}\frac{dz^\top}{dy}
  $$

  ### 泰勒展开

x,u 为向量, H(x,u)为标量函数, 在(x0,u0)处展开
$$
H(x,u)=H(x_0,u_0)+\left(\frac{\partial H}{\partial x}\right)^\top \delta x 
+ \left(\frac{\partial H}{\partial u}\right)^\top \delta u +
\frac{1}{2}
\begin{bmatrix}
\delta x^\top&\delta u^\top
\end{bmatrix}
\begin{bmatrix}
\frac{\partial^2 H}{\partial x^2} & \frac{\partial^2 H}{\partial x \partial u}\\
\frac{\partial^2 H}{\partial u \partial x} & \frac{\partial^2 H}{\partial u^2}
\end{bmatrix}\begin{bmatrix}
\delta x\\\delta u
\end{bmatrix}\\
\delta x=x-x_0 \quad \delta u=u-u_0\\
\frac{\partial^2 H}{\partial u \partial x}=\left(\frac{\partial^2 H}{\partial x \partial u}\right)^\top
$$
F(x,u)为向量函数, 在(x0,u0)处展开
$$
F(x,u)=F(x_0,u_0)+\frac{\partial F}{\partial x} \delta x +\frac{\partial F}{\partial u} \delta u
$$
