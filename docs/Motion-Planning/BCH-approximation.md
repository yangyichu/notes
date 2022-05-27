## BCH 近似

#### cot 泰勒级数展开

$$
cot(x)=x^{-1}-\frac{1}{3}x-\frac{1}{45}x^3-\dots--\frac{2^{2n}B_n}{(2n)!}x^{2n-1}-\dots
$$

$$
B_n为伯努利数，B_0=1,B_1=-0.5,B_2=1/6，\dots
$$

伯努利数可以由多项式方程解出，第一项为1, 第二项为0.5或-0.5,之后所有奇数项(从0开始计数)都为0:  

$$
\sum_{i=0}^{n}C_{n+1}^iB_i=0
$$

## 李代数微扰直观写法

$$
\begin{align}
&\text{exp}(\Delta \phi^{\wedge})\text{exp}(\phi^{\wedge})\approx
\text{exp}((\phi+J_l^{-1}\Delta \phi)^{\wedge})\\
&\text{exp}(\phi^{\wedge})\text{exp}(\Delta \phi^{\wedge})\approx
\text{exp}((\phi+J_r^{-1}\Delta \phi)^{\wedge})\\
&\text{反之：}\\
&\text{exp}((\phi+\Delta \phi)^{\wedge})\approx\text{exp}((J_l\Delta \phi)^{\wedge})\text{exp}(\phi^{\wedge})
\\
&\text{exp}((\phi+\Delta \phi)^{\wedge})\approx\text{exp}(\phi^{\wedge})\text{exp}((J_r\Delta \phi)^{\wedge})
\end{align}
$$

