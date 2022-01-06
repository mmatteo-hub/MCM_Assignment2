# MOCOMLAB2

In this exercise we will define all the necessary functions to evaluate the manipulator Jacobian and then implement a simple Closed Loop Inverse Kinematics (CLIK) control.

**1.** Implement the `GetJacobianColumn(biTei, bTe, jointType)` function, which returns the Jacobian column for a generic joint i as the stacking of *J_A* (angular part) and *J_L* (linear
part), for both revolute and prismatic joint type.

**2.** Implement the `GetJacobian()` function which constructs and returns the whole Jacobian.

**3.** Define a goal transformation matrix and implement a function which evaluates the linear and angular error between the end effector and the goal. Then choose an angular gain
(*γ_a*) and a linear gain (*γ_l*), and evaluate the desired end effector velocity.

**4.** Finally applying the inverse kinematics formula to compute the corresponding desired joint velocities.

To verify that everything is working correctly we will provide you with a very simple simulator, to check that the robot’s end effector actually reaches the desired goal position.
Note: The functions developed in the last lab are needed also in this lab.
