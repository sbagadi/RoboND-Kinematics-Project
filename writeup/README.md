# Udacity Robotics Nanodegree - Robot Arm - Pick And Place Project

## Kinematic analysis

### Forward kinematics

To perform forward kinematics, we need to derive the following DH
parameters:

*  **Twist angle (<i>&alpha;<sub>i - 1</sub></i>)** - The angle between
   **<i>Z<sub>i - 1</sub></i>** and **<i>Z<sub>i</sub></i>** axes
   measured about the **<i>X<sub>i - 1</sub></i>** axis in a right-hand
   sense.
*  **Link length (<i>a<sub>i - 1</sub></i>)** - The distance between
   **<i>Z<sub>i - 1</sub></i>** and **<i>Z<sub>i</sub></i>** axes
   measured along the **<i>X<sub>i</sub></i>** axis.
*  **Link offset (<i>d<sub>i</sub></i>)** - The signed distance between
   **<i>X<sub>i - 1</sub></i>** and **<i>X<sub>i</sub></i>** axes
   measured along the **<i>Z<sub>i</sub></i>** axis.
*  **Joint variables (<i>&theta;<sub>i</sub></i>)** - The angle between
   **<i>X<sub>i - 1</sub></i>** and **<i>X<sub>i</sub></i>** axes
   measured about the **<i>Z<sub>i</sub></i>** axis in a right-hand
   sense.

**Figure**: Annotated robot arm

![Annotated robot arm](images/annotated_robo_arm.png)

The following table shows the DH parameter values obtained using the
URDF file.

**Table**: DH Parameters

|  i  | <i>&alpha;<sub>i - 1</sub></i> | <i>a <sub>i - 1</sub></i> | <i>d<sub>i</sub></i> | <i>&theta;<sub>i</sub></i>   |
| :-: | -----------------------------: | ------------------------: | -------------------: | ---------------------------: |
| 1   | 0                              | 0                         | 0.75                 | &theta;<sub>1</sub>          |
| 2   | - &pi;/2                       | 0.35                      | 0                    | &theta;<sub>2</sub> - &pi;/2 |
| 3   | 0                              | 1.25                      | 0                    | &theta;<sub>3</sub>          |
| 4   | - &pi;/2                       | - 0.054                   | 1.5                  | &theta;<sub>4</sub>          |
| 5   | &pi;/2                         | 0                         | 0                    | &theta;<sub>5</sub>          |
| 6   | - &pi;/2                       | 0                         | 0                    | &theta;<sub>6</sub>          |
| 7   | 0                              | 0                         | 0.303                | 0                            |

### Transformation matrices


