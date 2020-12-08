### Aubo I5 Modeling based on Screw Theory

#### Home Configuration

- with custom end effector

$$
-0.0405, -0.36325, 1.0085,0.500, -0.500, 0.500, 0.500
$$

- without custom end effector
  $$
    0.0,-0.2155,1.0085,\frac{\sqrt2}{2},0.0,-0.0,\frac{\sqrt{2}}{2}
  $$

#### Screw Axes

- In Space
  $$
  {S}^T = \begin{bmatrix}
  0&0&1&0&0&0\\
  0&-1&0&0&0.122&0\\
  0&1&0&-0.53&0&0\\
  0&-1&0&0.906&0&0\\
  0&0&1&-0.1215&0&0\\
  0&-1&0&1.0085&0&0
  \end{bmatrix}
  $$

- In Body

  - with custom end effector

  $$
  {B}^T = \begin{bmatrix}
  1&0&0&0&-0.36325&0.0405\\
  0&0&1&-0.0405&1.0085&-0.122\\
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
  0&0&1&-1.0085&0&-0.122\\
  0&0&-1&0.4785&0&0\\
  0&0&1&-0.1025&0&0\\
  0&1&0&0.094&0&0\\
  0&0&1&0&0&0
  \end{bmatrix}
  $$