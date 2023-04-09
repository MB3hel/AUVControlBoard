# Math

## Coordinate System Definition

The control board uses a coordinate system that is somewhat non-standard.

- The coordinate system is right handed
- `+y` is forward, `+z` is up, `+x` is right
- Pitch is about x, roll is about y, yaw is about z

<center>
![](./math_res/coord_system_1.png){: style="height:250px;"}
![](./math_res/coord_system_2.png){: style="height:250px;"}
![](./math_res/coord_system_rotations.png){: style="height:250px;"}
</center>

While this coordinate system may seem strange to some (especially anyone who has worked with planes), the name of axes doesn't really matter.

## 6 Degree of Freedom Motion Control


### Nomenclature & Convention Notes

1. Matrices are assumed to be zero indexed (not 1 indexed). This means the first element of a matrix $M$ is $m_{00}$ not $m_{11}$. This is because the math will be implemented in C (which uses zero indexed arrays).
2. Thruster numbers (1-8) are used by the control board's user facing components. However, this math will use thruster indices (0-7) where `index = number - 1`. This is b
3. The terms "pitch", "roll", and "yaw" are used to describe rotational DoFs, as well as orientation in 3D space. The context these words are used in should make it clear which is being used. "Pitch" is angular velocity about the x-axis, "roll" is angular rotation about the y-axis, and "yaw" is angular velocity about the z-axis.
4. Velocities in DoFs are normalized (meaning -1.0 to 1.0).


### System Assumptions

- Vehicle is capable of motion exclusively in each of 6 degrees of freedom (DoFs). These are three translational DoFs, and three rotational DoFs.
- Thruster orientations are fixed. Gimbaled thruster vehicles are not supported.
- At most 8 thrusters (less is fine)
- System has 3D orientation information
- System has depth information
- System does not have translational position information


### Example Vehicle

The examples on this page will use AquaPack robotics's SeaWolf VIII robot. This robot's thruster configuration is as shown below. The arrows indicate the direction a thruster moves water (meaning the arrows are opposite the direction the thruster excerpts force on the vehicle).

<center>
![](./math_res/thruster_config.png){: style="height:400px;"}
</center>

The arrows indicate the direction the thruster moves water when powered in the positive direction. These arrows are opposite the direction force is excerpted on the vehicle. Note that the diagram above uses thruster numbers, not indices.

### DoF Matrix

The *DoF Matrix*, $D$, is constructed based on the vehicle's thruster configuration. Rows of the matrix correspond to thrusters (by index). And columns of the matrix correspond to vehicle relative DoFs. Thus, this is an 8x6 matrix. Columns correspond to DoFs in the following order (0-5): x, y, z, pitch, roll, yaw. Note that pitch, roll, and yaw are written using the single variable notation p, r, h (h = heading = yaw).

$D = \left(\begin{array}{c|c|c|c|c|c} d_0 & d_1 & d_2  & d_3 & d_4 & d_5 \end{array}\right) = \left(\begin{array}{c|c|c|c|c|c} d_x & d_y & d_z  & d_p & d_r & d_h \end{array}\right)$

Each column of the DoF matrix, $d_i$ is a set of thruster speeds that result in motion *exclusively* in the column's DoF. Additionally, the resultant motion should be the maximum possible speed, and in the positive direction. Each $d_i$ is an 8 element column vector, with elements corresponding to thrusters (by index). All speeds should be normalized (between -1.0 and 1.0)

For the example vehicle shown above, the following is the DoF matrix

$D = 
\begin{pmatrix}
-1 & -1 & 0 & 0 & 0 & +1 \\
+1 & -1 & 0 & 0 & 0 & -1 \\
-1 & +1 & 0 & 0 & 0 & -1 \\
+1 & +1 & 0 & 0 & 0 & +1 \\
0 & 0 & -1 & -1 & -1 & 0 \\
0 & 0 & -1 & -1 & +1 & 0 \\
0 & 0 & -1 & +1 & -1 & 0 \\
0 & 0 & -1 & +1 & +1 & 0 \\
\end{pmatrix}$

Consider the first column: $d_0 = d_x$. This column's thruster speeds should result in the vehicle moving as fast as possible in the +x direction (only). This is achieved by setting T2, T4 (index 1, 3) to the positive direction and T1, T3 (index 0, 2) to the negative direction at full speed (recall that the arrows are opposite the direction the thruster excerpts force on the vehicle). Thus

$d_x = \begin{pmatrix}-1 & +1 & -1 & +1 & 0 & 0 & 0 & 0\end{pmatrix}^T$

Note that when constructing the DoF matrix for your vehicle, you should assume an ideal system and environment.


### LOCAL Mode Motion

In LOCAL mode, motion is specified as a set of speeds in vehicle relative DoFs. The user provides the control board with a *target motion vector* ($t_l$) where each element corresponds to a DoF.

$t_l = \begin{pmatrix} x & y & z & p & r & h \end{pmatrix}^T$

$x$ is normalized velocity in +x direction  
$y$ is normalized velocity in +y direction  
$z$ is normalized velocity in +y direction  
$p$ is normalized velocity in +pitch direction  
$r$ is normalized velocity in +roll direction  
$h$ is normalized velocity in +yaw direction

By multiplying this target motion by the DoF matrix, $D$, a *speed vector* $s$ is obtained where each element of $s$ corresponds to a specific thruster (by index).

$s = D t_l$

Consider the example where $t_l = \begin{pmatrix}0 & 1 & 0 & 0 & 0 & 0\end{pmatrix}^T$. This should cause the vehicle to move at full possible speed forward (relative to the vehicle's orientation).

$s = D t_l = 
\begin{pmatrix}
-1 & -1 & 0 & 0 & 0 & +1 \\
+1 & -1 & 0 & 0 & 0 & -1 \\
-1 & +1 & 0 & 0 & 0 & -1 \\
+1 & +1 & 0 & 0 & 0 & +1 \\
0 & 0 & -1 & -1 & -1 & 0 \\
0 & 0 & -1 & -1 & +1 & 0 \\
0 & 0 & -1 & +1 & -1 & 0 \\
0 & 0 & -1 & +1 & +1 & 0 \\
\end{pmatrix}
\begin{pmatrix}0 \\ 1 \\ 0 \\ 0 \\ 0 \\ 0\end{pmatrix}
=
\begin{pmatrix}-1 \\ -1 \\ +1 \\ +1 \\ 0 \\ 0 \\ 0 \\ 0\end{pmatrix}$

In the above example, it is trivial to see that this is the desired motion. However for a more complex example, a problem appears. Consider $t_l = \begin{pmatrix}0 & 1 & 0 & 0 & 0 & 1\end{pmatrix}^T$. This describes the vehicle both moving forward and yawing at full possible speed.

$s = D t_l = 
\begin{pmatrix}
-1 & -1 & 0 & 0 & 0 & +1 \\
+1 & -1 & 0 & 0 & 0 & -1 \\
-1 & +1 & 0 & 0 & 0 & -1 \\
+1 & +1 & 0 & 0 & 0 & +1 \\
0 & 0 & -1 & -1 & -1 & 0 \\
0 & 0 & -1 & -1 & +1 & 0 \\
0 & 0 & -1 & +1 & -1 & 0 \\
0 & 0 & -1 & +1 & +1 & 0 \\
\end{pmatrix}
\begin{pmatrix}0 \\ 1 \\ 0 \\ 0 \\ 0 \\ 1\end{pmatrix}
=
\begin{pmatrix}0 \\ -2 \\ 0 \\ +2 \\ 0 \\ 0 \\ 0 \\ 0\end{pmatrix}$

Notice that the resultant speed vector has motors moving in excess of 100% speed (elements with magnitude greater than 1.0). This is not possible. While a simple solution may seem to be dividing all elements of the vector by the one with the largest magnitude. This results in a scaled speed vector $\hat{s}$

$\hat{s} = s \div \text{absmax}(s)$

this will not work well in all cases. Consider $t_l = \begin{pmatrix}0 & 1 & 1 & 1 & 1 & 1\end{pmatrix}^T$.

$s = D t_l = 
\begin{pmatrix}
-1 & -1 & 0 & 0 & 0 & +1 \\
+1 & -1 & 0 & 0 & 0 & -1 \\
-1 & +1 & 0 & 0 & 0 & -1 \\
+1 & +1 & 0 & 0 & 0 & +1 \\
0 & 0 & -1 & -1 & -1 & 0 \\
0 & 0 & -1 & -1 & +1 & 0 \\
0 & 0 & -1 & +1 & -1 & 0 \\
0 & 0 & -1 & +1 & +1 & 0 \\
\end{pmatrix}
\begin{pmatrix}0 \\ 1 \\ 1 \\ 1 \\ 1 \\ 1\end{pmatrix}
=
\begin{pmatrix}0 \\ -2 \\ 0 \\ +2 \\ -3 \\ -1 \\ -1 \\ +1\end{pmatrix}$

and

$\hat{s} = s \div \text{absmax}(s) = s \div 3 = \begin{pmatrix}0 \\ -0.67 \\ 0 \\ +0.67 \\ -1 \\ -0.33 \\ -0.33 \\ +0.33\end{pmatrix}$

While this has resulted in an possible set of thruster speeds, these are not optimal. Look at the example vehicle diagram. Notice that thrusters 1-4 and 5-8 control different motions. In the previous example, thrusters 1-4 were slowed down more than necessary, because thruster 5 was too large of a value. This is not ideal as the vehicle's maximum speed becomes artificially limited. Instead, the following $\hat{s}$ is ideal. This is scaling down the thrusters within each group (1-4 and 5-8) separately.

$\hat{s} = \begin{pmatrix}0 \\ -1 \\ 0 \\ +1 \\ -1 \\ -0.33 \\ -0.33 \\ +0.33\end{pmatrix}$

Groupings of thrusters on the example vehicle are easy to observe, however this is not always true. Thus, achieving optimal scaling for any system (any DoF matrix) requires a more sophisticated method to determine groupings and scale speeds.

Thruster groupings are determined by "overlap" between thrusters. Two thrusters, $i$ and $j$ are said to overlap if they have a non-zero entry in the same column of the DoF matrix ($D$) for at least one column. This is easier to calculate using a *contribution matrix*, $C$, defined as $D \neq 0$. This results in a binary form of the DoF matrix. For the above example

$C = 
\left[\begin{pmatrix}
-1 & -1 & 0 & 0 & 0 & +1 \\
+1 & -1 & 0 & 0 & 0 & -1 \\
-1 & +1 & 0 & 0 & 0 & -1 \\
+1 & +1 & 0 & 0 & 0 & +1 \\
0 & 0 & -1 & -1 & -1 & 0 \\
0 & 0 & -1 & -1 & +1 & 0 \\
0 & 0 & -1 & +1 & -1 & 0 \\
0 & 0 & -1 & +1 & +1 & 0 \\
\end{pmatrix}
\neq 0 \right] = 
\begin{pmatrix}
1 & 1 & 0 & 0 & 0 & 1 \\
1 & 1 & 0 & 0 & 0 & 1 \\
1 & 1 & 0 & 0 & 0 & 1 \\
1 & 1 & 0 & 0 & 0 & 1 \\
0 & 0 & 1 & 1 & 1 & 0 \\
0 & 0 & 1 & 1 & 1 & 0 \\
0 & 0 & 1 & 1 & 1 & 0 \\
0 & 0 & 1 & 1 & 1 & 0 \\
\end{pmatrix}$

Then for each thruster $i$ an overlap vector $o_i$ can be constructed as follows

$o_i = C (c^i)^T$

where $c^i$ is the $i$th row of $C$. Thus, $o_i$ is an 8 element vector where each element corresponds to a thruster (by index). Element $j$ of $o_i$ can either be $1$ or a $0$. $1$ indicates that thrusters $i$ and $j$ overlap.

For example, 

$o_0 = C (C^0)^T = \begin{pmatrix} 1 & 1 & 1 & 1 & 0 & 0 & 0 & 0 \end{pmatrix}^T$

This shows that thruster index 0 (T1) overlaps with indices 0, 1, 2, and 3 (T1, T2, T3, T4).

By calculating and storing these overlap vectors for each thruster ($\left\{o_i\right\}_{i=0}^7$), this effectively forms a lookup table to determine thruster overlap. While this is not the most memory efficient option, it reduces computation time, which is important since this will run very frequently on a microcontroller.

Using overlap vectors, the following algorithm can be used to scale motor speeds:

- Find the thruster with the largest magnitude speed
- Iterate over that thruster's overlap vector
- For any thruster it overlaps with, divide speed by the largest speed
- Repeat until the largest magnitude does not exceed 1.0

```
while true
    // m is value, i is index
    m, i = max(abs(speed_vector))
    if m <= 1.0
        // Done scaling
        break
    endif

    // Iterate over all thrusters (0-7 inclusive)
    for j=0...7
        if overlap_vector[i][j] == 1
            // i and j overlap. Divide j's speed by m.
            speed_vector[j] /= m
        endif
    endfor
endwhile
```

This algorithm results in optimal speed scaling by only reducing the speed of thrusters that share DoF contributions.


