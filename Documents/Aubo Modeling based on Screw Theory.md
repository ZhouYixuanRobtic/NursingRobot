###                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             Aubo I5 Modeling based on Screw Theory

​	To simplify analytical inverse my_kinematics solution, we should keep the screw axes as simple as possible. Fortunately, we have a very regular (most axes are parallel) robot, so we construct our screw axes referring to the first joint, rather than base. For perhaps usage, we also perform axes, as well as home configuration referring to the base.

#### Home Configuration

In case of singularity brought by Euler angles or misunderstanding caused by different representation type of Euler angles, we adopt a compact representation (position and quaternion) rather than a matrix representation.

- Robot Home Configuration

  Robot Home Configuration refers to the pose from **Joint 1** to **Joint 6**, which remains constant because of rigid mechanical structure.
  $$
  ^1M_6 = -0.000, 0.2155, 0.8865,0.000, \sqrt2/2, \sqrt2/2, 0.000
  $$
  we refers 0.2155=0.1215+0.094, 0.8865 = 0.408+0.376+0.1025

- Mount Configuration

  Mount Configuration refers to the pose from **Base/World** to **Joint  1**
  $$
  ^sM_1 = 0.000, 0.000, 0.122,0,0,1,0
  $$

- End Effector Configuration

  End Effector Configuration refers to the pose from **Joint 6** to **End Effector**
  $$
  ^6M_E = -0.0405, 0.0, 0.14775,0.000, -0.000, \sqrt2/2, \sqrt2/2
  $$

- Planning Configuration (used in forward my_kinematics and numerical inverse my_kinematics, if you want)

$$
^SM_E= {}^SM_1{}^1M_6 {}^6M_E  = -0.0405, -0.36325, 1.0085,0.500, -0.500, 0.500, 0.500
$$

- Inverse Configuration (used in analytical inverse my_kinematics)
  $$
  ^1M_E={}^1M_6 {}^6M_E  = 0.0405, 0.36325, 0.8865,0.500, 0.500, 0.500, -0.500
  $$

#### Screw Axes

- In Body
  $$
  {B_E} = Ad({^{E}M_1}) S = Ad({}^EM_S)S_M
  $$

  $$
  B = Ad({}^6M_1)S = Ad({}^6M_S)S_M
  $$

  - with custom end effector

  $$
  {B_E}^T = \begin{bmatrix}
  1&0&0&0&-0.36325&0.0405\\
  0&0&1&-0.0405&0.8865&0\\
  0&0&-1&0.0405&-0.4785&0\\
  0&0&1&-0.0405&0.1025&0\\
  1&0&0&0&-0.24175&0.0405\\
  0&0&1&-0.0405&0&0
  \end{bmatrix}
  $$

  

  - without custom end effector
    $$
    {B}^T = \begin{bmatrix}
    0&1&0&0.2155&0&0\\
    0&0&1&0&-0.8865&0\\
    0&0&-1&0.4785&0&0\\
    0&0&1&-0.1025&0&0\\
    0&1&0&0.094&0&0\\
    0&0&1&0&0&0
    \end{bmatrix}
    $$

- In Space
  $$
  {S_M} = Ad({}^SM_E)(Ad({^{E}M_1}) S)
  $$

  $$
  S =Ad({}^1M_E)(Ad({^{E}M_S}) S_M)
  $$

  - with custom mount link

  $$
  {S_M}^T = \begin{bmatrix}
  0&0&1&0&0&0\\
  0&-1&0&0.122&0&0\\
  0&1&0&-0.53&0&0\\
  0&-1&0&0.906&0&0\\
  0&0&1&-0.1215&0&0\\
  0&-1&0&1.0085&0&0
  \end{bmatrix}
  $$

  - without custom mount link
    $$
    {S}^T = \begin{bmatrix}
    0&0&1&0&0&0\\
    0&1&0&0&0&0\\
    0&-1&0&0.408&0&0\\
    0&1&0&-0.784&0&0\\
    0&0&1&0.1215&0&0\\
    0&1&0&-0.8865&0&0
    \end{bmatrix}
    $$

#### Inverse Kinematics


​	For simplifying the inverse my_kinematics, the screw axes and home configuration used here are $S$ and ${}^1M_6$

- Matrix form of ${}^1M_6$
  $$
  {}^1M_6 = \begin{bmatrix}
  -1&0&0&0\\
  0&0&1&0.2155\\
  0&1&0&0.8865\\
  0&0&0&1
  \end{bmatrix} = \begin{bmatrix}
  -1&0&0&0\\
  0&0&1&h2\\
  0&1&0&d_1+d_2+d_3\\
  0&0&0&1
  \end{bmatrix}
  $$
  where$h_1 = 0.1215,h_2=0.2155,$, $d_1 =0.408,d_2=0.376,d_3=0.1025$

- Analytic matrix form of $e^{S_i\theta_i}$
  $$
  e^{S_1\theta_1} = \begin{bmatrix}
  c_1&-s_1&0&0\\
  s_1&c_1&0&0\\
  0&0&1&0\\
  0&0&0&1
  \end{bmatrix}
  $$

  $$
  e^{S_2\theta_2} = \begin{bmatrix}
  c_2&0&s_2&0\\
  0&1&0&0\\
  -s_2&0&c_2&0\\
  0&0&0&1
  \end{bmatrix}
  $$

$$
e^{S_3\theta_3} = \begin{bmatrix}
c_3&0&-s_3&d_1s_3\\
0&1&0&0\\
s_3&0&c_3&(1-c_3)d_1\\
0&0&0&1
\end{bmatrix}
$$

$$
e^{S_4\theta_4} = \begin{bmatrix}
c_4&0&s_4&-s_4(d_1+d_2)\\
0&1&0&0\\
-s_4&0&c_4&(d_1+d_2)(1-c_4)\\
0&0&0&1
\end{bmatrix}
$$

$$
e^{S_5\theta_5} = \begin{bmatrix}
c_5&-s_5&0&s_5h_1\\
s_5&c_5&0&h_1(1-c_5)\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}
$$

$$
e^{S_6\theta_6} = \begin{bmatrix}
c_6&0&s_6&-s_6(d_1+d_2+d_3)\\
0&1&0&0\\
-s_6&0&c_6&(d_1+d_2+d_3)*(1-c_6)\\
0&0&0&1
\end{bmatrix}
$$

- Algorithm

  ![image-20201208222819406](C:\Users\zhouy\AppData\Roaming\Typora\typora-user-images\image-20201208222819406.png)

  Input: desired pose $M_d$

  Warning: Sometimes homogenous form and non-homogenous form are mixed, please note!

  - Step 1:

    Calculate $M_1 = M_d * {}^1M_6^{-1}$

    Denotes the intersection point of sixth and fifth joints is $p_{56}$,
    $$
    p_{56} =[0,h1,d_1+d_2+d_3,1]^T
    $$
    Calculate $p_1 = M_1 * p_{56}$

    For analytical purpose, denoting $p_1 = [p_x,p_y,p_z,1]^T$

  - Step 2:  Solving $\theta_1$

  ​	   Calculate $o_1 = \omega_1\omega_1^Tp_1 = [0,0,p_z]$
  $$
  \begin{equation}
  \left\{\begin{array}{c}
  \omega_2^T\left(p_{2}-p_{56}\right)=0 \\
  \omega_1^T\left(p_{2}-o_{1}\right)=0 \\
  \left\|p_{2}-o_{1}\right\|=\left\|p_{1}-o_{1}\right\|
  \end{array}\right.
  \end{equation}
  $$
  ​		so $p_2$ can be obtained using equation (22)
  $$
  p_2 =[\pm \sqrt{p_{x}^{2}+p_{y}^{2}-h_{1}^{2}},h_1,p_z]^T
  $$
  ​		Construct basic Paden-Kahan subproblem 1, and denotes $r_1 = \sqrt{p_{x}^{2}+p_{y}^{2}-h_{1}^{2}}$
  $$
  e^{S_1\theta_1}.p_2 = p_1
  $$

  
  $$
  \theta_1 =atan2(p_y(\pm r_1)-h_1p_x,p_yh_1 \pm p_xr_1)
  $$

  - Step 3: Solving $\theta_5, \theta_6$

    Denotes:
    $$
    \begin{equation}
    e^{-S_{1} \theta_{1}} \cdot M_1=e^{S_{2} \theta_{2}} e^{S_{3}\theta_{3}} e^{S_{4} \theta_{4}} e^{S_{5} \theta_{5}} e^{S_{6} \theta_{6}}
    \end{equation} = M_2
    $$
    where, $M_1 = M_d {{}^1M_6}^{-1}$
    
    So marks all elements  in $M_d$ as $a_{ij} \{i,j = 0...3\}$
    
    we have
    $$
    \begin{equation}
    \left\{\begin{array}{c}
    c_6s_5 = a_{00}s_1 - a_{10}c_1 = r_2 \\
    c_5 = a_{12}c_1 - a_{02}s_1 = r_3 \\
    s_5s_6 = a_{11}c_1-a_{01}s_1 = r_4 \\
    \end{array}\right.
    \end{equation}
    $$
    so
    $$
    \begin{array}{c}
    \theta_5 = \pm acos(r_3)\\
    \theta_6 = atan2(r4/s_5,r_2/s_5)\\
    \end{array}
    $$
    Warning, when $s_5 = 0$, the solution of $\theta_6$ does not exist. So be careful with the singularity condition $r_3 = \pm1$. When $\theta_5 = \pm \pi/0$, additional information (reference of $\theta_6$ ) is needed to full determine the full system.
    
  - Step 4: Solving $\theta_2, \theta_3, \theta_4$
  
    Denotes:
    $$
  M_2e^{-S_{6} \theta_{6}}e^{-S_{5} \theta_{5}} =e^{S_{2} \theta_{2}} e^{S_{3}\theta_{3}} e^{S_{4} \theta_{4}} =M_3
    $$
    Marks all elements in $M_3$ as $b_{ij}{i,j=0...3}$
  
    We have
    $$
  \begin{array}{c}
    d_1s_2+d_2s_{2-3} = b_{03}+(d_2+d_1)s_{2-34} = r_6\\
    d_1c_2+d_2c_{2-3} = b_{23} + (d_2+d_1)c_{2-34} = r_7\\
    s_{2-34} = b_{02};\\
    c_{2-34} = b_{00};
    \end{array}
    $$
    so 
    $$
    \begin{array}{c}
    2d_1d_2c_3 = r_6^2+r_7^2-d_1^2-d_2^2\\
    \frac{r_6}{\sqrt{r_6^2+r_7^2}}s_2+\frac{r_7}{\sqrt{r_6^2+r_7^2}}c_2 = \frac{r_6^2+r_7^2+d_1^2-d_2^2}{2d_1\sqrt{r_6^2+r_7^2}}
    \end{array}
    $$
    so
    $$
    \begin{array}{c}
    \theta_3 = \pm acos(\frac{r_6^2+r_7^2-d_1^2-d_2^2}{2d_1d_2})\\
    \theta_2 = \pm acos(\frac{r_6^2+r_7^2+d_1^2-d_2^2}{2d_1\sqrt{r_6^2+r_7^2}})+atan2(r_6,r_7)\\
    \theta_2 -\theta_3+\theta_4 = atan2(b_{02},b_{00})
    \end{array}
    $$
  
- 

Warning, when $|r_6^2+r_7^2-d_1^2-d_2^2| > 1$ has singularity

​		So
$$
\theta_4 = (\theta_2-\theta_3+\theta_4)-\theta_2+\theta_3
$$

- Test:

  Forward my_kinematics:

  wrist singular when $\theta_5 = \pm \pi/0$

