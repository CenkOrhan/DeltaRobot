# DeltaRobot
## Inverse Kinematics for Delta Robot

To express the inverse kinematics of the delta robot in formulaic form, let’s break down the key calculations done in the function `deltaCaltAngleYZ`. This function computes angles in a specific YZ-plane, while `deltaCaltUbverse` performs coordinate transformations to rotate by ±120° around the Z-axis, obtaining the three joint angles (delta angles) for the robot's configuration.

### 1. Parameters and Variables
- \( x_0, y_0, z_0 \): Target coordinates of the end effector.
- \( \text{rf} \): Length of the upper arm.
- \( \text{re} \): Length of the lower arm.
- \( f \): Length of the platform on which the end-effector is mounted.
- \( e \): Length of the base platform.

### 2. Derivation of Equations

#### Step 1: Calculating Offset in the Y-Direction
Calculate the offsets \( y_1 \) and \( y_0 \):
\[
y_1 = -0.5 \times 0.57735 \times f
\]
\[
y_0 = y_0 - 0.5 \times 0.57735 \times e
\]
This shift along the \( y \)-axis is based on the delta robot’s geometry. \( 0.57735 \approx \tan(30^\circ) \), which corresponds to the robot's 30° base symmetry.

#### Step 2: Calculating Parameters \( a \) and \( b \) for the Joint Angle Equation
The values \( a \) and \( b \) are calculated as intermediate parameters to help in finding the joint position:
\[
a = \frac{x_0^2 + y_0^2 + z_0^2 + \text{rf}^2 - \text{re}^2 - y_1^2}{2 \cdot z_0}
\]
\[
b = \frac{y_1 - y_0}{z_0}
\]
These values help in calculating the angle within the YZ plane.

#### Step 3: Discriminant and Calculation of \( y_j \) and \( z_j \)
Calculate the discriminant \( d \) to ensure a real solution exists:
\[
d = -\left(a + b \cdot y_1\right)^2 + \text{rf} \cdot \left(b^2 \cdot \text{rf} + \text{rf}\right)
\]
If \( d < 0 \), no real solution exists, and the point cannot be reached.

If \( d \geq 0 \), we proceed to calculate \( y_j \) and \( z_j \):
\[
y_j = \frac{y_1 - a \cdot b - \sqrt{d}}{b^2 + 1}
\]
\[
z_j = a + b \cdot y_j
\]
These are the joint coordinates in the YZ-plane.

#### Step 4: Calculating the Angle \(\delta\)
Calculate the angle \(\delta\) using the arctangent of \(-z_j / (y_1 - y_j)\) and convert it to degrees:
\[
\delta = \left(\frac{180}{\pi} \cdot \arctan\left(-\frac{z_j}{y_1 - y_j}\right) + \left( y_j > y_1 ? 180 : 0 \right) \right) \times i
\]
This gives the angle the arm makes in the YZ plane.

### 3. Calculating the Three Angles for Rotated Coordinates
The function `deltaCaltUbverse` applies `deltaCaltAngleYZ` to three rotated versions of the target coordinates (0°, +120°, -120° around the Z-axis). These rotations are done using Cosine and Sine values for 120°:
\[
\text{For } \delta_2: \quad x_0 \cos(120^\circ) + y_0 \sin(120^\circ), \quad y_0 \cos(120^\circ) - x_0 \sin(120^\circ)
\]
\[
\text{For } \delta_3: \quad x_0 \cos(120^\circ) - y_0 \sin(120^\circ), \quad y_0 \cos(120^\circ) + x_0 \sin(120^\circ)
\]

### Summary
The three angles \(\delta_1\), \(\delta_2\), and \(\delta_3\) are obtained from these calculations, representing the joint angles needed to position the end effector at \((x_0, y_0, z_0)\).
