part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Calibration:

Use [intrinsic_calib.cc](https://github.com/dvorak0/camera_model/blob/master/src/intrinsic_calib.cc) to calibrate your camera.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)




## Fisheye camera introduce

Fisheye camera is a kind of camera with a fisheye lens, which peovide a wide angle filed of view. The FOV of fisheye mentioned here are almost more than 180 degrees.

![nikon-fisheyecamera][1]

## Camera projection

For pinhole camera, the projection function can be:
$$
\begin{bmatrix} x\\y\\1 \end{bmatrix} = f \begin{bmatrix} X/Z \\Y/Z \\ 1\end{bmatrix} \\
$$This equation means that the relationship between the incident ray and the projection hight is:
$$ h = f tan(\theta) $$
Well, in this model, for the incoming light, the angle $\theta$ is limited by the tangent function:
$$ \lim_{\theta = 90^{\circ}} tan(\theta) = \infty$$So that $\theta<90 ^{\circ} $ and the FOV is less than $180^{\circ}$.

As for the fisheye cameras, all of them have more than $180^{\circ}$ FOV. So we need to use another model to discribe the light projection by the special lens.

First of all, the image hight formulation becomes:
$$
h = f \theta
$$ Specially, the unit of the focal length $f$ become $mm/^{\circ}$ , $pixel/^{\circ}$, $mm/rad$ or $pixal/rad$. And base on the lens design software ZEMAX, the distortion data of a lens is available from the manufacturers. Use the $f-\theta$ distortion plot, the error can be discrebe.

Regard that $h_{ref} = f\theta$ is the refrence image hight calcalated by the ideal fisheye lens projection. And the $h$ is the measured real image hight projection by the lens (the real image hight $h$ is calculated by the software simulate using accurate light tracking method, it can be regard as the ideal iamge of this lens).
Then the projection distortion error can be formulate as:
$$
h_{error} = h - h_{ref}
$$
With the error fomulation, define:
$$
d = \frac{h_{error}}{h_{ref}} \times 100\%
$$
The $d$ is called the "F-Theta" distortion, and the data of each lens design are difrent, and can be provided by the ZEMAX or other software, from the mamufacturers.

Here is a example of the "F-Theta" distortion data:
![image_1b0fucrqn1ful1fo8dqn7kn11vp1j.png-164.9kB][2]

Draw the plot:
![image_1b0fugm5arbb18lh7pm1tcp1gmc2g.png-62.7kB][3]

And after draw a plot about the "F-Theta" distortion data relate by the imcoming light angle $\theta$, it is possible to fit the plot by a continous function $d(\theta)$.
Here, we regard a high order polynomial (with both odd order and even oder) to fit the "F-Theta" distortion data:
$$
d(\theta) = k_0+ k_1\theta + k_2\theta^2 + k_3\theta^3 + k_4\theta^4 + k_5\theta^5 + \cdots
$$
As for the plot showed before, a five order lopynomial can fit good enough:

![image_1b0g0tqlf16mr2k1kth11ic1u8o9.png-36.8kB][4]

If the plot is fitted well enough, for a real lens, the error can only emerged by the error and mistake during production.

---

## Fisheye camera Projection

With the "F-Theta" distortion polynomial, the $d(\theta)$ is accurate enough.
$$
d(\theta) = \frac{h - h_{ref}}{h_{ref}}= \frac{h - f\theta}{f\theta} $$
So for a incoming light,
$$
h = [1+d(\theta)]h_{ref} = [1+d(\theta)]\theta f $$

If regard $d(\theta)$ is a polynomial about $\theta$, $[1+d(\theta)]\theta$ is also a polynomial about $\theta$, as $\theta_{cali}(\theta)$:

$$
D(\theta) = h_{non-scale} = \theta_{calib}(\theta) = [1+d(\theta)]\theta ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\\
= \theta[ (1+k_0)+ k_1\theta + k_2\theta^2 + k_3\theta^3 + k_4\theta^4 + k_5\theta^5 + \cdots ] \\
= (1+k_0)\theta+ k_1\theta^2 + k_2\theta^3 + k_3\theta^4 + k_4\theta^5 + k_5\theta^6 + \cdots \\
= a_0\theta+a_1\theta^2+a_2\theta^3+a_3\theta^4+a_4\theta^5+a_5\theta^6+\cdots~~~~~~~~~
$$
Specially, the polynomial coeffient of $d(\theta)$, $k_0,\cdots, k_n$, is konwn by "F-Theta" polynomial fit. The initial value of polynomial coeffient of $\theta_{calib}(\theta)$, $a_0,\cdots, a_n$, can also be calculated.

For a real point in the camera frame $X_c =[x_c~y_c~z_c]^T$.
###Step 1. $X_c\rightarrow X_s$:
First step is to let a real point become a incident ray from the point to the object center. Map the point onto a unit sphere and ignore the scalar infomation of the point, means to normalize the vector $X_c$:
$$ X_s = \frac{X_c}{||X_c||} $$ 

###Step 2. $X_s\rightarrow [\theta,\varphi]$:
This step is to transform the normarlized vector to angular representation, get the direction of the incoming ray.
The incident angle is the angle between incoming ray and the z-axis of camera coordinate.
$$
cos(\theta) = X_s \cdot
\begin{bmatrix}
0\\0\\1
\end{bmatrix} = X_{sz}~~~\theta\in [0,\pi)
$$

$$
tan(\varphi) = \frac{X_{sy}}{X_{sx}}~~~\varphi\in(-\pi,0)\cup(0, \pi)$$
specially,if $X_{sx}=0$,  


###Step 3. $[\theta,\varphi]\rightarrow X_i$:
This step is to use the direction of the incident ray and the distortion function to calculate the normalized image coordinate. That means divide the non-scale image hight into x and y component.
$$ \begin{bmatrix}
X_{ix}\\
X_{iy}
\end{bmatrix}=
\mathit{D}(\theta)
\begin{bmatrix}
cos(\varphi) \\ sin(\varphi)
\end{bmatrix}
$$Here, known that $D(\theta)=[1+d(\theta)].$

###Step 4. $X_i\rightarrow [u,v]$
This step is to change the image point from the normalized image coordinates to pixel coordinates. It is known as:
$$
\begin{bmatrix}
u\\v 
\end{bmatrix}=
\begin{bmatrix}
f_x & 0\\
0 & f_y
\end{bmatrix}
\begin{bmatrix}
X_{ix}\\
X_{iy}
\end{bmatrix} +
\begin{bmatrix}
c_x\\c_y 
\end{bmatrix}
$$ Actually, because of the rotaion between the lens and CMOS chip, this step will became an affine transformation:
$$
\begin{bmatrix}
u\\v 
\end{bmatrix}=
A_f
\begin{bmatrix}
X_{ix}\\
X_{iy}
\end{bmatrix} +
\begin{bmatrix}
c_x\\c_y 
\end{bmatrix}
$$

Above all, the whole fisheye camera projection is discribe as:
$$
\begin{bmatrix}
u\\v 
\end{bmatrix}=
[1+d(\theta)]\theta
\begin{bmatrix}
f_x & 0\\
0 & f_y
\end{bmatrix}
\begin{bmatrix}
cos(\varphi) \\ sin(\varphi)
\end{bmatrix} +
\begin{bmatrix}
c_x\\c_y
\end{bmatrix}
$$

##Fisheye camera backward model

With the image coordinate, to calculate the direction of the incident ray.

$$
\begin{bmatrix}
x\\y \end{bmatrix}
=
A_f^{-1}
\begin{pmatrix}
\begin{bmatrix}
u\\v \end{bmatrix} -
\begin{bmatrix}
c_x\\c_y \end{bmatrix}
\end{pmatrix} \\
cos\varphi = \frac{x}{r} \\
sin\varphi = \frac{y}{r} \\
$$

Then to calculate the root of the polinomial $D(\theta)$:
$$ D(\theta) = r = \sqrt{x^2+y^2} \\
 \theta = D^{-1}(r) = root(r) $$
So the direction vector of the incident ray is:
$$
X_s = \begin{bmatrix}
sin\theta cos\varphi \\
sin\theta sin\varphi \\
cos\theta
\end{bmatrix}
$$  It also the coordinate of the project point on the unit sphere.
 
 
 

  [1]: http://www.pierretoscani.com/images/echo_fisheyes/Figure-18.jpg
  [2]: http://static.zybuluo.com/gaowenliang/t04cyfsxc10qz4576imjr1ik/image_1b0fucrqn1ful1fo8dqn7kn11vp1j.png
  [3]: http://static.zybuluo.com/gaowenliang/pm7tvvpidw26cnfz3b1kc8b5/image_1b0fugm5arbb18lh7pm1tcp1gmc2g.png
  [4]: http://static.zybuluo.com/gaowenliang/7rr5d2ple1nj4tnn0hh02j38/image_1b0g0tqlf16mr2k1kth11ic1u8o9.png
