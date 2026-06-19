In this work we will try to do the following steps for whole understanding of a free-floating Space Manipulator System (SMS)
1. First we will derive the EOM for the SMS
2. Forward Kinematics will be verified
3. Conservation of momentum will be tested
4. Joint space and Cartesian space (task space) PID control will be designed
5. Model based Sliding mode control that takes into account the dynamic coupling will be derived and implemented


\textbf{Derivation of Equation of Motion of Space Manipulator System:}
\section{Frame Transformation}
Let $B$ be the satellite base coordinate frame and $N$ be the inertial coordinate frame. Any vector ${}^B\mathbf{r}$ expressed in the base frame can be transformed to the inertial frame as
\begin{equation}
    {}^N\mathbf{r} = C_{NB} {}^B\mathbf{r}\blue{,}
\end{equation}
where $C_{NB}$ is the direction cosine matrix (DCM) defined as
\begin{equation}
    C_{NB} = R_x(\alpha_b) R_y(\theta_b) R_z(\psi_b)
    \label{Eq_DCM}\blue{.}
\end{equation}
In Eq. (\ref{Eq_DCM}), $\alpha_b$, $\theta_b$, and $\psi_b$ are the base satellite's roll, pitch, and yaw angles, respectively. $R_x \in \Re^{3\times 3}$ is the rotation matrix about $x$-axis, $R_y \in \Re^{3\times 3}$ is the rotation matrix about $y$-axis, and $R_z \in \Re^{3\times 3}$ is the rotation matrix about $z$-axis. {Note that the top-left superscript indicates the frame information where the elements of the vector are expressed.} The inertia matrix of base satellite ${}^BI_b$ {with respect to its center of mass} can be transformed to the inertial frame as \cite{wilde2018equations}
