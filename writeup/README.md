# Udacity Robotics Nanodegree - Robot Arm - Pick And Place Project

## Kinematic analysis

### Forward kinematics

To perform forward kinematics, we need to obtain the following DH
parameters:

*  **Twist angle (*&alpha;<sub>i - 1</sub>*)** - The angle between
   ***Z<sub>i - 1</sub>*** and ***Z<sub>i</sub>*** axes measured about
   the ***X<sub>i - 1</sub>*** axis in a right-hand sense.
*  **Link length (*a<sub>i - 1</sub>*)** - The distance between
   ***Z<sub>i - 1</sub>*** and ***Z<sub>i</sub>*** axes measured along
   the ***X<sub>i</sub>*** axis.
*  **Link offset (*d<sub>i</sub>*)** - The signed distance between
   ***X<sub>i - 1</sub>*** and ***X<sub>i</sub>*** axes measured along
   the ***Z<sub>i</sub>*** axis.
*  **Joint variables (*&theta;<sub>i</sub>*)** - The angle between
   ***X<sub>i - 1</sub>*** and ***X<sub>i</sub>*** axes measured about
   the ***Z<sub>i</sub>*** axis in a right-hand sense.

**Figure**: Annotated robot arm

![Annotated robot arm](images/annotated_robo_arm.png)

The following table shows the DH parameter values obtained using the
URDF file.

**Table**: DH Parameters

|  i  | *&alpha;<sub>i - 1</sub>* | *a <sub>i - 1</sub>* | *d<sub>i</sub>* | *&theta;<sub>i</sub>*        |
| :-: | ------------------------: | -------------------: | --------------: | ---------------------------: |
| 1   | 0                         | 0                    | 0.75            | &theta;<sub>1</sub>          |
| 2   | - &pi;/2                  | 0.35                 | 0               | &theta;<sub>2</sub> - &pi;/2 |
| 3   | 0                         | 1.25                 | 0               | &theta;<sub>3</sub>          |
| 4   | - &pi;/2                  | - 0.054              | 1.5             | &theta;<sub>4</sub>          |
| 5   | &pi;/2                    | 0                    | 0               | &theta;<sub>5</sub>          |
| 6   | - &pi;/2                  | 0                    | 0               | &theta;<sub>6</sub>          |
| G   | 0                         | 0                    | 0.303           | 0                            |

### Transformation matrices

To transform the reference frame from joint *i - 1* to joint *i* four
transformations need to be performed (2 rotations and 2 translations) in
the following order:

1. Rotate about the *X* axis by an angle of *&alpha;<sub>i - 1</sub>*.
2. Translate along the *X* axis by a length of *a<sub>i - 1</sub>*.
3. Rotate about the *Z* axis by an angle of *&theta;<sub>i</sub>*.
4. Translate about the *Z* axis by a length of *d<sub>i</sub>*.

The total transformation from frame *i - 1* to *i* can be written as

***<sup>i - 1</sup><sub>i</sub> T =
R<sub>X</sub>(&alpha;<sub>i - 1</sub>)
D<sub>X</sub>(a<sub>i - 1</sub>)
R<sub>Z</sub>(&theta;<sub>i</sub>)
D<sub>X</sub>(d<sub>i</sub>)***

The resulting Homogeneous Transformation matrix is

***<sup>i - 1</sup><sub>i</sub> T =&#9;\[\[ &#9;cos(&theta;<sub>i</sub>), &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;-sin(&theta;<sub>i</sub>), &#9;&#9;0, &#9; &nbsp; &nbsp;a<sub>i - 1</sub> ]*** <br>
&#9; ***\[ sin(&theta;<sub>i</sub>)cos(&alpha;<sub>i - 1</sub>), &nbsp; cos(&theta;<sub>i</sub>)cos(&alpha;<sub>i - 1</sub>), &#9; &nbsp; &nbsp; -sin(&alpha;<sub>i - 1</sub>), &nbsp; &nbsp; &nbsp;-sin(&alpha;<sub>i - 1</sub>) d<sub>i</sub>]*** <br>
&#9; ***\[ sin(&theta;<sub>i</sub>)cos(&alpha;<sub>i - 1</sub>), &nbsp; cos(&theta;<sub>i</sub>)cos(&alpha;<sub>i - 1</sub>),&#9; &nbsp; &nbsp; cos(&alpha;<sub>i - 1</sub>), &nbsp; &nbsp; &nbsp; cos(&alpha;<sub>i - 1</sub>) d<sub>i</sub>]*** <br>
&#9; ***\[ &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &#9;&nbsp; &nbsp;0, &#9;&#9;0, &#9;&nbsp; &nbsp;&nbsp; &nbsp; &nbsp; 1]]*** <br>

By substituting the values for *&theta;<sub>i</sub>*,
*a<sub>i - 1</sub>*, *&alpha;<sub>i - 1</sub>* and *d<sub>i</sub>* with
the values in the DH parameters  for each link we get the following
simplified matrices:

***<sup>0</sup><sub>1</sub>T = &#9;\[\[ cos(&theta;<sub>1</sub>), &#9;-sin(&theta;<sub>1</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ sin(&theta;<sub>1</sub>), &#9;cos(&theta;<sub>1</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;1, &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0.75]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;0,    &#9;1]]*** <br>

***<sup>1</sup><sub>2</sub>T = &#9;\[\[ sin(&theta;<sub>2</sub>), &#9;cos(&theta;<sub>2</sub>), &#9;0, &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0.35]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;1, &#9;0]*** <br>
***&#9; \[ cos(&theta;<sub>2</sub>), &#9;-sin(&theta;<sub>2</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;0,    &#9;1]]***

***<sup>2</sup><sub>3</sub>T = &#9;\[\[ cos(&theta;<sub>3</sub>), &#9;-sin(&theta;<sub>3</sub>), &#9;0, &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;1.25]*** <br>
***&#9; \[ sin(&theta;<sub>3</sub>), &#9;cos(&theta;<sub>3</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;1, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;0,    &#9;1]]*** <br>

***<sup>3</sup><sub>4</sub>T = &#9;\[\[  cos(&theta;<sub>4</sub>), &#9;-sin(&theta;<sub>4</sub>), &#9;0, &nbsp; &nbsp; &nbsp;-0.054]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9;1,  &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 1.5]*** <br>
***&#9; \[-sin(&theta;<sub>4</sub>),&nbsp;-cos(&theta;<sub>4</sub>), &#9;0,&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;0,    &#9;1]]*** <br>

***<sup>4</sup><sub>5</sub>T = &#9;\[\[ cos(&theta;<sub>5</sub>), -sin(&theta;<sub>5</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;-1, &#9;0]*** <br>
***&#9; \[ sin(&theta;<sub>5</sub>), &nbsp;cos(&theta;<sub>5</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;0,    &#9;1]]*** <br>

***<sup>5</sup><sub>6</sub>T = &#9;\[\[ cos(&theta;<sub>6</sub>), -sin(&theta;<sub>6</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;1, &#9;0]*** <br>
***&#9; \[ -sin(&theta;<sub>6</sub>),-cos(&theta;<sub>6</sub>), &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;0,    &#9;1]]*** <br>

***<sup>6</sup><sub>G</sub>T = &#9;\[\[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 1, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 1, &#9;0, &#9;0]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;1, &nbsp; &nbsp; &nbsp; &nbsp;0.303]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;0,    &#9;1]]*** <br>

#### Generalized homogeneous transform between base_link and gripper_link

The transform from the base link to the gripper can be obtained using
the roll, pitch and yaw values in the gripper pose by multiplying the
rotation matrices R<sub>X</sub>(roll), R<sub>Y</sub>(pitch) and
R<sub>Z</sub>(yaw).

***T<sub>rpy</sub> = &#9;\[\[ &#9; &#9; &nbsp; cos(P)\*cos(Y), &#9; &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;-sin(Y)\*cos(P), &#9;sin(P), &#9; P<sub>x</sub>]*** <br>
***&#9; \[ sin(P)\*sin(R)\*cos(Y) + sin(Y)\*cos(R), &nbsp; -sin(P)\*sin(R)\*sin(Y) + cos(R)\*cos(Y), &nbsp; &nbsp;-sin(R)\*cos(P), &#9; P<sub>y</sub>]*** <br>
***&#9; \[-sin(P)\*cos(R)\*cos(Y) + sin(R)\*sin(Y), &nbsp; sin(P)\*sin(Y)\*cos(R) + sin(R)\*cos(Y), &nbsp; &nbsp; &nbsp;cos(P)\*cos(R), &#9; P<sub>z</sub>]*** <br>
***&#9; \[ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9; &nbsp; &nbsp; &nbsp; &nbsp; 0, &#9;0,    &#9;1]]*** <br>

The rotation still needs to be corrected to align it with the gripper
frame. The resulting frame from the transform above should be rotated
intrinsically by *-&pi;/2* along the *Y* axis and then by *&pi;* radians
along the *Z* axis.

***T<sub>corr</sub> = R<sub>Z</sub>(&pi;) \* R<sub>Y</sub>(-&pi;/2)***

***T<sub>corr</sub> = &#9;\[\[ 0, &#9;0, &#9;1, &#9;0]*** <br>
***&#9; \[ 0, &#9;-1, &#9;0, &#9;0]*** <br>
***&#9; \[ 1, &#9;0, &#9;0, &#9;0]*** <br>
***&#9; \[ 0, &#9;0, &#9;0,    &#9;1]]*** <br>

the generalized transform is given by
***T<sub>total</sub> = T<sub>rpy</sub> \* T<sub>corr</sub>*** .

***T<sub>total</sub> = &#9;\[\[ &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; sin(P), &#9; &#9; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; sin(Y)\*cos(P), &#9; &#9; &#9; cos(P)\*cos(Y), &#9; P<sub>x</sub>]*** <br>
***&#9; \[ sin(R)\*cos(P), &nbsp; sin(P)\*sin(R)\*sin(Y) - cos(R)\*cos(Y), &nbsp; sin(P)\*sin(R)\*cos(Y) + sin(Y)\*cos(R), &#9; P<sub>y</sub>]*** <br>
***&#9; \[cos(P)\*cos(R), &nbsp;-sin(P)\*sin(Y)\*cos(R) - sin(R)\*cos(Y), &nbsp;-sin(P)\*cos(R)\*cos(Y) + sin(R)\*sin(Y), &#9; P<sub>z</sub>]*** <br>
***&#9; \[ &#9; &nbsp; &nbsp;0, &#9; &#9; &#9;&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;0, &#9; &#9; &#9; &#9; &nbsp; 0, &#9; &nbsp;1]]*** <br>

### Inverse Kinematics


