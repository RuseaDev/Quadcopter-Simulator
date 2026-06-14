# Rotational Matrices

$$
T_3(\psi) =
\begin{bmatrix}
\cos\psi & \sin\psi & 0 \\
-\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix},
$$

$$
T_2(\theta) =
\begin{bmatrix}
\cos\theta & 0 & -\sin\theta \\
0 & 1 & 0 \\
\sin\theta & 0 & \cos\theta
\end{bmatrix},
$$


$$
T_1(\phi) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & \sin\phi \\
0 & -\sin\phi & \cos\phi
\end{bmatrix}
$$

# Angular Velocity in Body Frame

$$
\boldsymbol{\omega} =
\boldsymbol{\omega}_\phi +
\boldsymbol{\omega}_\theta +
\boldsymbol{\omega}_\psi
$$

- $\dot{\phi}$ : angular velocity of the $x$ axis
- $\dot{\theta}$ : angular velocity of the intermediate $y'$ axis
- $\dot{\psi}$ : angular velocity of the inertial $Z$ axis

Angular Velocity in Body Frame:

$$
\boldsymbol{\omega}_{b} = 

\boldsymbol{\omega}_{\psi,b} + 
\boldsymbol{\omega}_{\theta,b} + 
\boldsymbol{\omega}_{\phi, b}
$$

Mapped using Rotationla Matrices:

$$
\boldsymbol{\omega}_b =
T_1(\phi)T_2(\theta)
\begin{bmatrix}
0 \\
0 \\
\dot{\psi}
\end{bmatrix} +
T_1(\phi)
\begin{bmatrix}
0 \\
\dot{\theta} \\
0
\end{bmatrix} +
\begin{bmatrix}
\dot{\phi} \\
0 \\
0
\end{bmatrix}
$$

## Yar Angular Rate : $\boldsymbol{\omega}_{\psi,b}$

$$
T_2(\theta)
\begin{bmatrix}
0 \\
0 \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
c_\theta & 0 & -s_\theta \\
0 & 1 & 0 \\
s_\theta & 0 & c_\theta
\end{bmatrix}
\begin{bmatrix}
0 \\
0 \\
\dot{\psi}
\end{bmatrix} =
\begin{bmatrix}
-\dot{\psi}s_\theta \\
0 \\
\dot{\psi}c_\theta
\end{bmatrix}
$$

$$
\boldsymbol{\omega}_{\psi,b} = 
T_1(\phi)T_2(\theta)
\begin{bmatrix}
0 \\
0 \\
\dot{\psi}
\end{bmatrix} =
T_1(\phi)
\begin{bmatrix}
-\dot{\psi}s_\theta \\
0 \\
\dot{\psi}c_\theta
\end{bmatrix} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & c_\phi & s_\phi \\
0 & -s_\phi & c_\phi
\end{bmatrix}
\begin{bmatrix}
-\dot{\psi}s_\theta \\
0 \\
\dot{\psi}c_\theta
\end{bmatrix}
$$

$$
\boldsymbol{\omega}_{\psi,b} =
\begin{bmatrix}
-\dot{\psi}\sin\theta \\
\dot{\psi}\cos\theta\sin\phi \\
\dot{\psi}\cos\theta\cos\phi
\end{bmatrix}
$$

## Pitch Angular Rate : $\boldsymbol{\omega}_{\theta,b}$

$$
\boldsymbol{\omega}_{\theta,b}

= 

T_1(\phi)
\begin{bmatrix}
0 \\
\dot{\theta} \\
0
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & c_\phi & s_\phi \\
0 & -s_\phi & c_\phi
\end{bmatrix}
\begin{bmatrix}
0 \\
\dot{\theta} \\
0
\end{bmatrix}
$$

$$
\boldsymbol{\omega}_{\theta,b}
=
\begin{bmatrix}
0 \\
\dot{\theta}\cos\phi \\
-\dot{\theta}\sin\phi
\end{bmatrix}
$$

## Roll Angular Rate : $\boldsymbol{\omega}_{\phi,b}$
$$
\boldsymbol{\omega}_{\psi,b}

= 

\begin{bmatrix}
\dot{\psi}\\
0\\
0
\end{bmatrix}
$$

## Angular Velocity : 
$$ 
\boldsymbol{\omega}_{b} 
= 
\boldsymbol{\omega}_{\phi,b} 
+
\boldsymbol{\omega}_{\theta,b} 
+
\boldsymbol{\omega}_{\psi,b} 
$$

$$
\boldsymbol{\omega}_{b} 
= 
\begin{bmatrix}
-\dot{\psi}\sin\theta \\
\dot{\psi}\cos\theta\sin\phi \\
\dot{\psi}\cos\theta\cos\phi
\end{bmatrix}
+
\begin{bmatrix}
0 \\
\dot{\theta}\cos\phi \\
-\dot{\theta}\sin\phi
\end{bmatrix}
+
\begin{bmatrix}
\dot{\psi}\\
0\\
0
\end{bmatrix}
$$



$$
\boxed{
\begin{aligned}
\omega_x
&= -\dot{\psi}\sin\theta + 0 + \dot{\phi}
= \dot{\phi} - \dot{\psi}\sin\theta, \\
\omega_y
&= \dot{\psi}\cos\theta\sin\phi + \dot{\theta}\cos\phi + 0 
= \dot{\theta}\cos\phi + \dot{\psi}\cos\theta\sin\phi, \\
\omega_z
&= \dot{\psi}\cos\theta\cos\phi - \dot{\theta}\sin\phi + 0
= -\dot{\theta}\sin\phi + \dot{\psi}\cos\theta\cos\phi.
\end{aligned}
}
$$

## Solve for $ {\dot{\theta}} $

Multiply the $\omega_y$ with $\cos\phi$:

$$
\omega_y\cos\phi
=
\dot{\theta}\cos^2\phi
+
\dot{\psi}\cos\theta\sin\phi\cos\phi.
$$

Multiply the $\omega_z$ with $-\sin\phi$:

$$
-\omega_z\sin\phi
=
\dot{\theta}\sin^2\phi
-
\dot{\psi}\cos\theta\cos\phi\sin\phi.
$$

Add two equations, we have:

$$
\omega_y\cos\phi - \omega_z\sin\phi
=
\dot{\theta}\cos^2\phi
+
\dot{\theta}\sin^2\phi 
+
(\dot{\psi}\cos\theta\sin\phi\cos\phi
-
\dot{\psi}\cos\theta\cos\phi\sin\phi)
$$

$$
\omega_y\cos\phi - \omega_z\sin\phi
= \dot{\theta}\left(\cos^2\phi + \sin^2\phi\right)
$$

$$
\omega_y\cos\phi - \omega_z\sin\phi
= \dot{\theta}
$$

Hence,

$$
\boxed{
\dot{\theta}
=
\omega_y\cos\phi - \omega_z\sin\phi
}
$$

## Solve for : $ \dot{\psi} $

Multiply the $\omega_y$ with $\sin\phi$:

$$
\omega_y\sin\phi
=
\dot{\theta}\cos\phi\sin\phi
+
\dot{\psi}\cos\theta\sin^2\phi.
$$

Multiply the $\omega_z$ with $\cos\phi$:

$$
\omega_z\cos\phi
=
-\dot{\theta}\sin\phi\cos\phi
+
\dot{\psi}\cos\theta\cos^2\phi.
$$

Add 2 equations:

$$
\omega_y\sin\phi + \omega_z\cos\phi
=
(\dot{\theta}\cos\phi\sin\phi
-
\dot{\theta}\sin\phi\cos\phi)
+
\dot{\psi}\cos\theta\sin^2\phi
+
\dot{\psi}\cos\theta\cos^2\phi.
$$

$$
\omega_y\sin\phi + \omega_z\cos\phi
=
\dot{\psi}\cos\theta
\left(\sin^2\phi + \cos^2\phi\right)
$$

$$
\omega_y\sin\phi + \omega_z\cos\phi
=
\dot{\psi}\cos\theta.
$$

Assume $\cos\theta \neq 0$, divide function with $\cos\theta$:

$$
\dot{\psi}
=
\frac{\omega_y\sin\phi + \omega_z\cos\phi}{\cos\theta}
$$

$$
\boxed{
\dot{\psi}
=
\left(\omega_y\sin\phi + \omega_z\cos\phi\right)\sec\theta
}
$$

## Solve for $ \dot{\phi} $ 

We have:

$$
\omega_x = \dot{\phi} - \dot{\psi}\sin\theta,
$$

add $\dot{\psi}\sin\theta$ to both sides:

$$
\dot{\phi}
=
\omega_x + \dot{\psi}\sin\theta.
$$

Substitute $\dot{\psi} = \omega_y\sin\phi + \omega_z\cos\phi$ to the equation:

$$
\dot{\phi}
=
\omega_x
+
\left(\omega_y\sin\phi 
+ 
\omega_z\cos\phi\right)
\sec\theta\sin\theta
$$

Because:

$$
\sec\theta\sin\theta
=
\frac{1}{\cos\theta}\sin\theta
=
\tan\theta.
$$

The equation becomes:

$$
\boxed{
\dot{\phi}
=
\omega_x
+
\left(\omega_y\sin\phi 
+ \omega_z\cos\phi\right)\tan\theta
}
$$

## Mapping Rotational Matrix

We have: 
$$
\begin{aligned}
\dot{\theta}
&=
\omega_y\cos\phi - \omega_z\sin\phi \\

\dot{\psi}
&=
\left(\omega_y\sin\phi + \omega_z\cos\phi\right)\sec\theta \\

\dot{\phi}
&=
\omega_x
+
\left(\omega_y\sin\phi 
+ \omega_z\cos\phi\right)\tan\theta

\end{aligned}
$$

Thereby, the matrix to map angular velocity to body frame is:

$$
\boxed{
\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi\sec\theta & \cos\phi\sec\theta
\end{bmatrix}
\begin{bmatrix}
\omega_x \\
\omega_y \\
\omega_z
\end{bmatrix}
}
$$