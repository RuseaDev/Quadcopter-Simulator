## Velocity Function

We have:

$$
\mathbf{F}_{total} = \mathbf{F}_{gravity\_world} + \mathbf{F}_{thrust\_world}
$$

And: 

$$
\begin{aligned}
&\mathbf{F}_{total} = m \mathbf{a}\\

&\mathbf{F}_{gravity\_world} = m \mathbf{g}
\end{aligned}
$$

Hence, the function becomes:

$$
m \mathbf{a} = m \mathbf{g} + \mathbf{F}_{thrust\_world}
$$

$$
\mathbf{a} = \frac{(\mathbf{F}_{thrust\_word})}{m} + \mathbf{g}
$$

$$
\boxed{
\dot{v} = \frac{(\mathbf{F}_{thrust\_word})}{m} + \mathbf{g}
}
$$

## Mapped from Body Frame

### Rotational Matrices
$$
T_3(\psi)
=
\begin{bmatrix}
\cos\psi & \sin\psi & 0 \\
-\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix},
$$

$$
T_2(\theta)
=
\begin{bmatrix}
\cos\theta & 0 & -\sin\theta \\
0 & 1 & 0 \\
\sin\theta & 0 & \cos\theta
\end{bmatrix},
$$


$$
T_1(\phi)
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & \sin\phi \\
0 & -\sin\phi & \cos\phi
\end{bmatrix}
$$

### Body to World Matrix
$$
\boxed{
\boldsymbol {R}_{b2w}
= 
T_3(\psi)T_2(\theta)T_1(\phi)}
$$

### Velocity Equation 
$$
\boxed{
\dot{v}
= 
\frac{1}{m} (\boldsymbol{R}_{b2w} \mathbf{F}_{thrust\_body}) + \mathbf{g}
}
$$