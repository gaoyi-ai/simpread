> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/67872858)

> 最近在看 Joan Solà 大神的《Quaternion Kinematics for the error-state Kalman filter》，在此对其中的一些知识点进行总结，纯属搬书。

1. 四元数的定义与基本性质
--------------

**定义**
------

*   四元数的定义：1 个实部 + 3 个虚部

$$Q = q_w + q_x i + q_y j +q_z k \ \in \mathbb H \\$$

Q = q_w + q_x i + q_y j +q_z k \ \in \mathbb H \\

*   可以写成多种形式：

$$Q = q_w + \bm q_v =<q_w, \bm q_v> = [q_w, q_x, q_y, q_z]^T \\$$

Q = q_w + \bm q_v =<q_w, \bm q_v> = [q_w, q_x, q_y, q_z]^T \\

*   实部为 0 的四元数为纯虚四元数，虚部全为 0 的四元数即为实数。

性质
--

**Sum**

$$\bm p + \bm q = \left[\begin{array}{c} p_w + q_w\\ \bm p_v +\bm q_v\\ \end{array} \right] \\$$

\bm p + \bm q = \left[\begin{array}{c} p_w + q_w\\ \bm p_v +\bm q_v\\ \end{array} \right] \\

**Product**

$$\bm p \otimes \bm q = \left[ \begin{array}{c} p_w q_w - \bm p_v^T \bm q_v \\ p_w \bm q_v + q_w \bm p_v + \bm p_v \times \bm q_v \end{array} \right] \\$$

\bm p \otimes \bm q = \left[\begin{array}{c} p_w q_w - \bm p_v^T \bm q_v \\ p_w \bm q_v + q_w \bm p_v + \bm p_v \times \bm q_v \end{array} \right] \\

product not commutative: $\bm p \otimes \bm q \ne \bm q \otimes \bm p$\bm p \otimes \bm q \ne \bm q \otimes \bm p

但是 associativie: $(\bm p \otimes \bm q)\otimes \bm r =\bm p \otimes (\bm q \otimes \bm r)$ (\bm p \otimes \bm q)\otimes \bm r =\bm p \otimes (\bm q \otimes \bm r)

distributive over the sum:

$\bm p \otimes (\bm q +\bm r) = \bm p \otimes \bm q +\bm p \otimes \bm r \\ (\bm p +\bm q) \otimes \bm r = \bm p \otimes \bm r+ \bm q \otimes \bm r$ \bm p \otimes (\bm q +\bm r) = \bm p \otimes \bm q +\bm p \otimes \bm r \\ (\bm p +\bm q) \otimes \bm r = \bm p \otimes \bm r+ \bm q \otimes \bm r

product 可以转换为矩阵乘法：

$$\bm q_1 \otimes \bm q_2 = [\bm q_1]_L \bm q_2 = [\bm q_2]_R \bm q_1 \\$$

\bm q_1 \otimes \bm q_2 = [\bm q_1]_L \bm q_2 = [\bm q_2]_R \bm q_1 \\

$$[\bm q]_L = q_w \bm I + \left[ \begin{array}{cc} 0 && -\bm q_v^T \\ \bm q_v && [\bm q_v]_\times \\ \end{array} \right] \\ [\bm q]_R = q_w \bm I + \left[ \begin{array}{cc} 0 && -\bm q_v^T \\ \bm q_v && -[\bm q_v]_\times \\ \end{array} \right] \\ [\bm p]_R [\bm q]_L= [\bm q]_L [\bm p]_R \\$$

[\bm q]_L = q_w \bm I + \left[ \begin{array}{cc} 0 && -\bm q_v^T \\ \bm q_v && [\bm q_v]_\times \\ \end{array} \right] \\ [\bm q]_R = q_w \bm I + \left[ \begin{array}{cc} 0 && -\bm q_v^T \\ \bm q_v && -[\bm q_v]_\times \\ \end{array} \right] \\ [\bm p]_R [\bm q]_L= [\bm q]_L [\bm p]_R \\

corss-product matrix, 叉乘也可以转换为矩阵相乘：

$[\bm a]_\times = \left[\begin{array}{ccc} 0 && -a_z && a_y \\ a_z && 0 && -a_x \\ -a_y && a_x && 0 \\ \end{array} \right] \\ \bm a \times \bm b = [\bm a]_\times \bm b, \forall \bm a, \bm b \in \mathbb R^3$[\bm a]_\times = \left[\begin{array}{ccc} 0 && -a_z && a_y \\ a_z && 0 && -a_x \\ -a_y && a_x && 0 \\ \end{array} \right] \\ \bm a \times \bm b = [\bm a]_\times \bm b, \forall \bm a, \bm b \in \mathbb R^3

**Identity**

$$\bm q_1 = 1 = \left[\begin{array}{c} 1 \\ \bm 0_v \end{array} \right] \\$$

\bm q_1 = 1 = \left[\begin{array}{c} 1 \\ \bm 0_v \end{array} \right] \\

**Conjugate**

$\bm q^\ast = q_w - \bm q_v = \left[\begin{array}{c} q_w \\ -\bm q_v \end{array}\right] \\ \bm q^\ast \otimes \bm q = \bm q \otimes \bm q^\ast = q_w^2 +q_x^2 +q_y^2 + q_z^2 \\ (\bm p \otimes \bm q)^\ast = \bm q^\ast \otimes \bm p ^\ast$\bm q^\ast = q_w - \bm q_v = \left[\begin{array}{c} q_w \\ -\bm q_v \end{array}\right] \\ \bm q^\ast \otimes \bm q = \bm q \otimes \bm q^\ast = q_w^2 +q_x^2 +q_y^2 + q_z^2 \\ (\bm p \otimes \bm q)^\ast = \bm q^\ast \otimes \bm p ^\ast

**Norm**

$\|q\| = \sqrt {\bm q \otimes \bm q^\ast} = \sqrt{\bm q^\ast \otimes \bm q} = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2} \in \mathbb R \\ \|\bm p \otimes \bm q\| = \|\bm q \otimes \bm p \| = \|\bm p\| \|\bm q\|$\|q\| = \sqrt {\bm q \otimes \bm q^\ast} = \sqrt{\bm q^\ast \otimes \bm q} = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2} \in \mathbb R \\ \|\bm p \otimes \bm q\| = \|\bm q \otimes \bm p \| = \|\bm p\| \|\bm q\|

**Inverse**

$\bm q \otimes \bm q^{-1} = \bm q^{-1} \otimes \bm q_1 = \bm q_1 \\ \bm q^{-1} = \bm q^\ast / \|\bm q\|$\bm q \otimes \bm q^{-1} = \bm q^{-1} \otimes \bm q_1 = \bm q_1 \\ \bm q^{-1} = \bm q^\ast / \|\bm q\|

**Unit quaternion**

单位四元数的 Inverse 与 Conjugate 相同：

$\bm q^{-1} = \bm q^\ast \\ \bm q = \left[\begin{array}{c} cos \theta \\ \bm u sin \theta \end {array}\right].$\bm q^{-1} = \bm q^\ast \\ \bm q = \left[\begin{array}{c} cos \theta \\ \bm u sin \theta \end {array}\right].

我们用单位四元数来表示刚体旋转。

**几条特殊性质**

Quaternion commutator:

$\bm p \otimes \bm q - \bm q \otimes \bm p = 2 \bm p_v \times \bm q_v \\ \bm p_v \otimes \bm q_v - \bm q_v \otimes \bm p_v = 2 \bm p_v \otimes \bm q_v$\bm p \otimes \bm q - \bm q \otimes \bm p = 2 \bm p_v \times \bm q_v \\ \bm p_v \otimes \bm q_v - \bm q_v \otimes \bm p_v = 2 \bm p_v \otimes \bm q_v

Product of pure quaternions

对于纯四元数：$\bm p_v \otimes \bm q_v = -\bm p_v^T \bm q_v + \bm p_v \times \bm q_v = \left[\begin{array}{c} -\bm p_v^T \bm q_v \\ \bm p_v \times \bm q_v \end{array}\right] \\ 对于纯四元数： \bm q_v \otimes \bm q_v = - \|\bm q_v\|^2$\bm p_v \otimes \bm q_v = -\bm p_v^T \bm q_v + \bm p_v \times \bm q_v = \left[\begin{array}{c} -\bm p_v^T \bm q_v \\ \bm p_v \times \bm q_v \end{array}\right] \\ 对于纯四元数： \bm q_v \otimes \bm q_v = - \|\bm q_v\|^2

natural powers of pur quaternions， 用于推到指数函数的泰勒展开

$\bm v = \bm u \theta \\ \bm v^2 = -\theta^2; \bm v^3 = -\bm u \theta^3; \bm v^4 = \theta^4; \bm v^5 = \bm u \theta^5; \bm v^6 = -\theta^6.$\bm v = \bm u \theta \\ \bm v^2 = -\theta^2; \bm v^3 = -\bm u \theta^3; \bm v^4 = \theta^4; \bm v^5 = \bm u \theta^5; \bm v^6 = -\theta^6.

Exponential of pure quaternions，指数函数的泰勒展开：

$e^{\bm v} = e^{\bm u \theta} = \sum_{k=0}^{\infty}\frac{1}{k!} \bm v^k=\left(1 - \frac{\theta^2}{2!} + \frac {\theta^4} {4!}+\cdots\right) + \left(\bm u \theta - \frac{\bm u \theta^3}{3!} + \frac{\bm u \theta^5}{5!} + \cdots\right) \\ = cos\theta + \bm u sin\theta$e^{\bm v} = e^{\bm u \theta} = \sum_{k=0}^{\infty}\frac{1}{k!} \bm v^k=\left(1 - \frac{\theta^2}{2!} + \frac {\theta^4} {4!}+\cdots\right) + \left(\bm u \theta - \frac{\bm u \theta^3}{3!} + \frac{\bm u \theta^5}{5!} + \cdots\right) \\ = cos\theta + \bm u sin\theta

纯虚四元数的指数函数为单位四元数。

Exponential of general quaternions:

$e^\bm q = e^{q_w + \bm q_v} = e^{q_w}e^{\bm q_v} \\ = e^{q_w} \left[\begin{array}{c} cos {\|q_v\|} \\ \frac{\bm q_v}{\|q_v\|} sin{\|q_v\|} \end{array}\right]$ e^\bm q = e^{q_w + \bm q_v} = e^{q_w}e^{\bm q_v} \\ = e^{q_w} \left[\begin{array}{c} cos {\|q_v\|} \\ \frac{\bm q_v}{\|q_v\|} sin{\|q_v\|} \end{array}\right]

Logarithm of unit quaternions

$log \bm q = log(cos\theta +\bm u sin\theta)+ log(e^{\bm u \theta})=\bm u \theta = \left[\begin{array}{c} 0 \\ \bm u \theta \end{array}\right] \\ \bm u = \bm q_v / \|\bm q_v\|; \theta = arctan2(\|\bm q_v\|, q_w).$log \bm q = log(cos\theta +\bm u sin\theta)+ log(e^{\bm u \theta})=\bm u \theta = \left[\begin{array}{c} 0 \\ \bm u \theta \end{array}\right] \\ \bm u = \bm q_v / \|\bm q_v\|; \theta = arctan2(\|\bm q_v\|, q_w).

Logarithm of general quaternions:

$log(\bm q)=log(\|\bm q\| \frac{\bm q}{\|\bm q\|})= log(\|\bm q\|) +\log(\frac{\bm q}{\|\bm q\|}) \\ =log(\|\bm q\|) + \bm u \theta$log(\bm q)=log(\|\bm q\| \frac{\bm q}{\|\bm q\|})= log(\|\bm q\|) +\log(\frac{\bm q}{\|\bm q\|}) \\ =log(\|\bm q\|) + \bm u \theta

Exponential forms of the type of $q^t$q^t : $\bm q^t = exp(log(\bm q^t))=exp(t \cdot log(\bm q))$\bm q^t = exp(log(\bm q^t))=exp(t \cdot log(\bm q)) ，对于单位四元数：

$$\bm q^t = exp(t \cdot \bm u \theta) = \left[\begin{array}{c} cos t \theta \\ \bm u \sin t \theta \end{array}\right] \\$$

\bm q^t = exp(t \cdot \bm u \theta) = \left[\begin{array}{c} cos t \theta \\ \bm u \sin t \theta \end{array}\right] \\

2. 旋转的表达 Rotations and Cross-relations
--------------------------------------

The rotation group SO(3)，表达刚体旋转，需要满足三个性质：

*   Rotation preserves the vertor norm

$$\|r(\bm v)\|=\|\bm v\| \\$$

\|r(\bm v)\|=\|\bm v\| \\

*   Rotation preserves the angles between vectors

$$<r(\bm v), r(\bm w)> =<\bm v, \bm w>=\|\bm v\|\|\bm w\|cos\alpha \\$$

<r(\bm v), r(\bm w)> =<\bm v, \bm w>=\|\bm v\|\|\bm w\|cos\alpha \\

*   Rotation preserves the relative orientations of vectors

$$\bm u \times \bm v = \bm w \Leftrightarrow r(\bm u) \times r(\bm v) = r(\bm w) \\$$

\bm u \times \bm v = \bm w \Leftrightarrow r(\bm u) \times r(\bm v) = r(\bm w) \\

SO3 定义：

$$SO(3) : \left\{r: \mathbb R^3 \rightarrow \mathbb R^3 / \forall \bm v, \bm w \in \mathbb R^3, \|r(\bm v)\|=\|\bm v\|, r(\bm v)\times r(\bm w)=r(\bm v \times \bm w) \right\} \\$$

SO(3) : \left\{r: \mathbb R^3 \rightarrow \mathbb R^3 / \forall \bm v, \bm w \in \mathbb R^3, \|r(\bm v)\|=\|\bm v\|, r(\bm v)\times r(\bm w)=r(\bm v \times \bm w) \right\} \\

**SO3 的旋转矩阵表达**

$r(\bm v) = \bm R \bm v \\ (\bm R \bm v )^T(\bm R \bm v )=\bm v^T \bm v \\ \bm R^T \bm R = \bm I = \bm R \bm R^T \ orthogonal \\ \bm R^{-1} = \bm R^T \\ det (\bm R) = 1 \ special\\$ r(\bm v) = \bm R \bm v \\ (\bm R \bm v )^T(\bm R \bm v )=\bm v^T \bm v \\ \bm R^T \bm R = \bm I = \bm R \bm R^T \ orthogonal \\ \bm R^{-1} = \bm R^T \\ det (\bm R) = 1 \ special\\

*   The exponential map

$$\bm R = e^{[\bm v]_\times} \\ exp: \mathfrak{so}(3) \rightarrow SO(3); [\bm v]_\times \rightarrow exp([\bm v]_\times)\\$$

\bm R = e^{[\bm v]_\times} \\ exp: \mathfrak{so}(3) \rightarrow SO(3); [\bm v]_\times \rightarrow exp([\bm v]_\times)\\

*   The capitalized exponential map

$$Exp: \mathbb R^3 \rightarrow SO(3); \bm v \rightarrow Exp(\bm v)=e^{[\bm v]_\times} \\$$

Exp: \mathbb R^3 \rightarrow SO(3); \bm v \rightarrow Exp(\bm v)=e^{[\bm v]_\times} \\

*   Rotation matrix and rotation vector: the Rodrigues rotation formula

$\bm R = \bm I +sin \phi [\bm u]_\times +(1-cos\phi)[\bm u]_\times^2 \\ =\bm I cos\phi +[\bm u]_\times sin \phi + \bm u \bm u^T(1-cos\phi).$\bm R = \bm I +sin \phi [\bm u]_\times +(1-cos\phi)[\bm u]_\times^2 \\ =\bm I cos\phi +[\bm u]_\times sin \phi + \bm u \bm u^T(1-cos\phi).

*   The logarithmic maps

$log: SO(3) \rightarrow \mathfrak{so}(3); \bm R \rightarrow log(\bm R)=[\bm u \phi]_\times;\\ \phi = arccos\left(\frac{trace(\bm R)-1}{2}\right), \\ \bm u = \frac{(\bm R - \bm R^T)^\vee}{2sin\phi}$ log: SO(3) \rightarrow \mathfrak{so}(3); \bm R \rightarrow log(\bm R)=[\bm u \phi]_\times;\\ \phi = arccos\left(\frac{trace(\bm R)-1}{2}\right), \\ \bm u = \frac{(\bm R - \bm R^T)^\vee}{2sin\phi}

*   The capitalized logarithmic map

$Log: SO(3) \rightarrow \mathbb{R^3}; \bm R \rightarrow Log(\bm R) = \bm u \phi \\ Log(\bm R) = log(\bm R)^\vee$Log: SO(3) \rightarrow \mathbb{R^3}; \bm R \rightarrow Log(\bm R) = \bm u \phi \\ Log(\bm R) = log(\bm R)^\vee

**SO3(3) 的 Quaternion 表达**

*   The exponential map

$\bm q = e^{\bm V} \\ exp: \mathbb H_p \rightarrow S^3; \bm V \rightarrow exp(\bm V)$\bm q = e^{\bm V} \\ exp: \mathbb H_p \rightarrow S^3; \bm V \rightarrow exp(\bm V)

![](https://pic4.zhimg.com/v2-fef1f4bc20909781040be58a507709cb_r.jpg)

1.  The capitalized exponential map

$Exp: \mathbb R^3 \rightarrow S^3; \bm v = Exp(\bm v)= e^{\bm v/2} \\ \dot {\bm q} = \frac{1}{2} \bm q \otimes \bm \omega \\ \bm q = e^{\bm \omega t / 2}$Exp: \mathbb R^3 \rightarrow S^3; \bm v = Exp(\bm v)= e^{\bm v/2} \\ \dot {\bm q} = \frac{1}{2} \bm q \otimes \bm \omega \\ \bm q = e^{\bm \omega t / 2}

*   Quaternion and rotation vector

$$\bm q = Exp(\phi \bm u) = exp(\phi \bm u /2) =e^{\phi \bm u /2} = cos(\phi/2) +\bm u sin(\phi/2) = \left[\begin{array}{c} cos(\phi/2) \\ \bm u sin(\phi/2) \end{array}\right] \\$$

\bm q = Exp(\phi \bm u) = exp(\phi \bm u /2) =e^{\phi \bm u /2} = cos(\phi/2) +\bm u sin(\phi/2) = \left[\begin{array}{c} cos(\phi/2) \\ \bm u sin(\phi/2) \end{array}\right] \\

*   The logarithmic maps

$$log:S^3 \rightarrow \mathbb H_p; \bm q \rightarrow log(\bm q) = \bm u \theta \\ Log: S^3 \rightarrow \mathbb R^3; \bm q \rightarrow Log(\bm q) = \bm u \phi \\ Log(\bm q) = 2log(\bm q) \\ \phi = 2arctan(\|\bm q_v\|, q_w) \\ \bm u = \bm q_v / \|\bm q_v\| \\$$

log:S^3 \rightarrow \mathbb H_p; \bm q \rightarrow log(\bm q) = \bm u \theta \\ Log: S^3 \rightarrow \mathbb R^3; \bm q \rightarrow Log(\bm q) = \bm u \phi \\ Log(\bm q) = 2log(\bm q) \\ \phi = 2arctan(\|\bm q_v\|, q_w) \\ \bm u = \bm q_v / \|\bm q_v\| \\

*   Rotation matrix and quaternion，偷懒上个图吧。

![](https://pic4.zhimg.com/v2-78abc502349dfdf350f01239e778f58b_r.jpg)

*   Spherical linear interpolation (SLERP)

$$\bm q(t) = \bm q_0 \otimes(\bm q^{\ast} \otimes \bm q_1)^t = \bm q_0 \otimes \left[\begin{array}{c} cos(t \Delta \phi/2) \\ \bm u sin(t \Delta \phi/2) \end{array}\right] \\ \bm R(t) = \bm R_0 Exp(tLog(\bm R_0^T \bm R_1)) = \bm R_0(\bm R_0^T \bm R_1)^t \\$$

\bm q(t) = \bm q_0 \otimes(\bm q^{\ast} \otimes \bm q_1)^t = \bm q_0 \otimes \left[\begin{array}{c} cos(t \Delta \phi/2) \\ \bm u sin(t \Delta \phi/2) \end{array}\right] \\ \bm R(t) = \bm R_0 Exp(tLog(\bm R_0^T \bm R_1)) = \bm R_0(\bm R_0^T \bm R_1)^t \\

3. Quaternion conventions
-------------------------

Hamilton 右手系，JPL 左手系。

![](https://pic3.zhimg.com/v2-85e3408d446483ccb423f8949dea7142_r.jpg)

4. Perturbations, derivatives, and integrals.
---------------------------------------------

*   **SO3 的加减法定义**

The plus operator

$$S = \bm R \oplus \bm \theta = \bm R \circ Exp(\bm \theta)\ \ \ R,S \in SO(3), \bm \theta \in \mathbb R^3 \\ \bm q_s = \bm q_r \oplus \bm \theta = \bm q_r \otimes Exp(\bm \theta)\\ \bm R_s = \bm R_R \oplus \bm \theta = \bm R_R \cdot Exp(\bm \theta) \\$$

S = \bm R \oplus \bm \theta = \bm R \circ Exp(\bm \theta)\ \ \ R,S \in SO(3), \bm \theta \in \mathbb R^3 \\ \bm q_s = \bm q_r \oplus \bm \theta = \bm q_r \otimes Exp(\bm \theta)\\ \bm R_s = \bm R_R \oplus \bm \theta = \bm R_R \cdot Exp(\bm \theta) \\

The minus operator

$\bm \theta = \bm S \ominus \bm R = Log(\bm R^{-1} \bm S) \ \ \ \bm{R}, \bm S, \bm \theta \in \mathbb R^3 \\ \bm\theta = \bm q_s \ominus \bm q_R = Log(\bm q_R^{\ast} \otimes \bm q_s) \\ \bm \theta = \bm R_s \ominus \bm R_R = Log(\bm R_R^T \bm R_s) \\$ \bm \theta = \bm S \ominus \bm R = Log(\bm R^{-1} \bm S) \ \ \ \bm{R}, \bm S, \bm \theta \in \mathbb R^3 \\ \bm\theta = \bm q_s \ominus \bm q_R = Log(\bm q_R^{\ast} \otimes \bm q_s) \\ \bm \theta = \bm R_s \ominus \bm R_R = Log(\bm R_R^T \bm R_s) \\

*   **The four possible derivative definitions**

Functions from vector space to vector space

$\frac{\partial f(\bm x)}{\partial \bm x} = \mathop {\lim }\limits_{\delta x \to 0} \frac{f(\bm x +\delta \bm x) - f(\bm x)}{\delta \bm x} \\ f(\bm x +\Delta \bm x) \approx f(\bm x) + \frac{f(\bm x)}{\partial \bm x} \Delta\bm x$\frac{\partial f(\bm x)}{\partial \bm x} = \mathop {\lim }\limits_{\delta x \to 0} \frac{f(\bm x +\delta \bm x) - f(\bm x)}{\delta \bm x} \\ f(\bm x +\Delta \bm x) \approx f(\bm x) + \frac{f(\bm x)}{\partial \bm x} \Delta\bm x

Functions from SO(3) to SO(3)

$\frac{\partial f(\bm R)}{\partial \bm \theta} = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{f(\bm R \oplus \delta \bm \theta) \ominus f(\bm R)}{\delta \bm \theta} \\ = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{Log(f^{-1}(\bm R)f(\bm R Exp(\delta \bm \theta))}{\delta \bm \theta} \\ f(\bm R \oplus \Delta \bm \theta) \approx f(\bm R) Exp(\frac{\partial f(\bm R)}{\partial \bm \theta}\Delta \bm \theta)$\frac{\partial f(\bm R)}{\partial \bm \theta} = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{f(\bm R \oplus \delta \bm \theta) \ominus f(\bm R)}{\delta \bm \theta} \\ = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{Log(f^{-1}(\bm R)f(\bm R Exp(\delta \bm \theta))}{\delta \bm \theta} \\ f(\bm R \oplus \Delta \bm \theta) \approx f(\bm R) Exp(\frac{\partial f(\bm R)}{\partial \bm \theta}\Delta \bm \theta)

Functions from vector space to SO(3)

$$\frac{\partial f(\bm x)}{\partial \bm x} = \mathop {\lim }\limits_{\delta x \to 0} \frac{f(\bm x + \delta \bm x) \ominus f(\bm x)}{\delta \bm x} \\ \mathop {\lim }\limits_{\delta x \to 0} \frac{Log(f^{-1}(\bm x)f(\bm x + \delta \bm x))}{\delta \bm x} \\ f(\bm x + \Delta \bm x) \approx f(\bm x) Exp(\frac{\partial f(\bm x)}{\bm \partial x} \Delta \bm x)\\$$

\frac{\partial f(\bm x)}{\partial \bm x} = \mathop {\lim }\limits_{\delta x \to 0} \frac{f(\bm x + \delta \bm x) \ominus f(\bm x)}{\delta \bm x} \\ \mathop {\lim }\limits_{\delta x \to 0} \frac{Log(f^{-1}(\bm x)f(\bm x + \delta \bm x))}{\delta \bm x} \\ f(\bm x + \Delta \bm x) \approx f(\bm x) Exp(\frac{\partial f(\bm x)}{\bm \partial x} \Delta \bm x)\\

Function from SO(3) to vector space

$\frac{\partial f(\bm R)}{\partial \bm \theta} = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{f(\bm R \oplus \delta \bm \theta) - f(\bm R)}{\delta \bm \theta} \\ = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{f(\bm R Exp(\delta \bm \theta))-f(\bm R)}{\delta \bm \theta} \\ f(\bm R \oplus \Delta \bm \theta) \approx f(\bm R) + \frac{\partial f(\bm R)}{\partial \bm \theta}\Delta \bm \theta$\frac{\partial f(\bm R)}{\partial \bm \theta} = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{f(\bm R \oplus \delta \bm \theta) - f(\bm R)}{\delta \bm \theta} \\ = \mathop {\lim }\limits_{\delta \bm \theta \to 0} \frac{f(\bm R Exp(\delta \bm \theta))-f(\bm R)}{\delta \bm \theta} \\ f(\bm R \oplus \Delta \bm \theta) \approx f(\bm R) + \frac{\partial f(\bm R)}{\partial \bm \theta}\Delta \bm \theta

*   **Useful and very useful Jacobians of the rotations.**

Jacobian with respect to the vector

$$\frac{\partial (\bm q \otimes \bm a \otimes \bm q^{\ast})}{\partial \bm a} = \frac{\bm R \bm a}{\partial \bm a} = \bm R. \\$$

\frac{\partial (\bm q \otimes \bm a \otimes \bm q^{\ast})}{\partial \bm a} = \frac{\bm R \bm a}{\partial \bm a} = \bm R. \\

Jacobian with respect to the quaternion.

$$\frac{\partial (\bm q \otimes \bm a \otimes \bm q^{\ast})}{\partial \bm q} = 2 \left[\begin{array}{c} \omega \bm a + \bm v \times \bm a && | && \bm v^T \bm a \bm I +\bm v \bm a^T - \bm a \bm v^T - \omega [a]_\times \end{array}\right] \in \mathbb R^3. \\$$

\frac{\partial (\bm q \otimes \bm a \otimes \bm q^{\ast})}{\partial \bm q} = 2 \left[\begin{array}{c} \omega \bm a + \bm v \times \bm a && | && \bm v^T \bm a \bm I +\bm v \bm a^T - \bm a \bm v^T - \omega [a]_\times \end{array}\right] \in \mathbb R^3. \\

Right jacobian of SO(3)

$\bm J_r = \lim_{\delta \bm \theta \to \bm 0} \frac{Exp(\bm \theta + \delta \bm \theta) \ominus Exp(\bm \theta)}{\delta \bm \theta} \\ =\lim_{\bm \delta \theta \to 0} \frac{Log(Exp(\bm \theta)^TExp(\bm \theta +\delta \bm \theta))}{\delta \bm \theta}\ if\ using \ \bm R;\\ = \lim_{\bm \delta \theta \to 0} \frac{Log(Exp(\bm \theta)^{\ast} \otimes Exp(\bm \theta +\delta \bm \theta))}{\delta \bm \theta} \ \ if \ using \ \bm q.$ \bm J_r = \lim_{\delta \bm \theta \to \bm 0} \frac{Exp(\bm \theta + \delta \bm \theta) \ominus Exp(\bm \theta)}{\delta \bm \theta} \\ =\lim_{\bm \delta \theta \to 0} \frac{Log(Exp(\bm \theta)^TExp(\bm \theta +\delta \bm \theta))}{\delta \bm \theta}\ if\ using \ \bm R;\\ = \lim_{\bm \delta \theta \to 0} \frac{Log(Exp(\bm \theta)^{\ast} \otimes Exp(\bm \theta +\delta \bm \theta))}{\delta \bm \theta} \ \ if \ using \ \bm q.

可用来做一阶泰勒展开

$Exp(\bm \theta + \delta \bm \theta) \approx Exp(\bm \theta)Exp(\bm J_r(\bm \theta)\delta \bm \theta) \\ Exp(\bm \theta) Exp(\delta \bm \theta) \approx Exp(\bm \theta + \bm J_r^{-1}(\bm \theta)\delta \bm \theta) \\ Log(Exp(\bm \theta) Exp(\delta \bm \theta)) \approx \bm \theta + \bm J_r^{-1}(\bm \theta)\delta \bm \theta$ Exp(\bm \theta + \delta \bm \theta) \approx Exp(\bm \theta)Exp(\bm J_r(\bm \theta)\delta \bm \theta) \\ Exp(\bm \theta) Exp(\delta \bm \theta) \approx Exp(\bm \theta + \bm J_r^{-1}(\bm \theta)\delta \bm \theta) \\ Log(Exp(\bm \theta) Exp(\delta \bm \theta)) \approx \bm \theta + \bm J_r^{-1}(\bm \theta)\delta \bm \theta

Jacobian with respect to the rotation vector

$$\frac{\partial (\bm q\otimes \bm a \otimes \bm q^{\ast})}{\partial \bm a} = \frac{\partial\bm R\bm a}{\partial \bm a} = -\bm R \{\bm \theta\}[a]_\times \bm J_r(\bm \theta). \\$$

\frac{\partial (\bm q\otimes \bm a \otimes \bm q^{\ast})}{\partial \bm a} = \frac{\partial\bm R\bm a}{\partial \bm a} = -\bm R \{\bm \theta\}[a]_\times \bm J_r(\bm \theta). \\

*   **Time derivatives**

$$\dot {\bm q} = \frac 1 2 \Omega(\bm \omega_L)\bm q = \frac 1 2 \bm q \otimes \bm \omega_L; \ \ \ \ \ \ \dot{\bm R} = \bm R[\bm \omega_L]_\times. \\$$

\dot {\bm q} = \frac 1 2 \Omega(\bm \omega_L)\bm q = \frac 1 2 \bm q \otimes \bm \omega_L; \ \ \ \ \ \ \dot{\bm R} = \bm R[\bm \omega_L]_\times. \\

*   **Time-intergration of rotation rate**

**Zeroth order intergration**

Forward $\bm q_{n+1} \approx \bm q_n \otimes \bm q\{\bm \omega_n \Delta t\}.$ \bm q_{n+1} \approx \bm q_n \otimes \bm q\{\bm \omega_n \Delta t\}.

Backward $\bm q_{n+1} \approx \bm q_n \otimes \bm q\{\bm \omega_{n+1} \Delta t\}.$ \bm q_{n+1} \approx \bm q_n \otimes \bm q\{\bm \omega_{n+1} \Delta t\}.

Midward $\bar{\bm \omega} = \frac{\bm \omega_{n+1} + \bm \omega_{n}}{2}, \ \ \ \ \ \bm q_{n+1} \approx \bm q_n \otimes \bm q\{ \bar{\bm \omega} \Delta t\}.$ \bar{\bm \omega} = \frac{\bm \omega_{n+1} + \bm \omega_{n}}{2}, \ \ \ \ \ \bm q_{n+1} \approx \bm q_n \otimes \bm q\{ \bar{\bm \omega} \Delta t\}.

**First order integration**

$$\bm q_{n+1}\approx \bm q_n \otimes \left( \bm q\{\bar{\bm \omega} \Delta t\} + \frac{\Delta t^2}{24} \left[\begin{array}{c} 0 \\ \bm \omega_n \times \bm \omega_{n+1} \end{array}\right] \right) \\$$

\bm q_{n+1}\approx \bm q_n \otimes \left( \bm q\{\bar{\bm \omega} \Delta t\} + \frac{\Delta t^2}{24} \left[\begin{array}{c} 0 \\ \bm \omega_n \times \bm \omega_{n+1} \end{array}\right] \right) \\