# Distributed Planning of Multi-Agent Systems with Coupled Temporal Logic Specifications

In this paper, we investigate the multi-agent mission planning and control problem under
coordination constraints such as collision avoidance and spatial coherence. We propose a
sequential trajectory planning algorithm that is run by each agent in a distributed manner.
Mission requirements including servicing specific regions, avoiding static and dynamic obstacles
(other agents), and flying close to the other agents are incorporated into the problem as
Signal Temporal Logic (STL) constraints. The planning problem under STL constraints is
formulated as a Mixed Integer Linear Program and an off-the-shelf tool is used to find the
high-level trajectories. The resulting high-level trajectories are then tracked by the agents
via Model Predictive Control. Illustrative simulation and experiment results are presented in
addition to comparing the proposed method with a centralized solution.

The code features:
<ul type="square">
<!-- li><code>todo</code> </li -->
    <li>Plan the trajectories of multiple agents that achieve STL specifications coupled with other agents;</li>
    <li>Environment is defined in "main_scitech.m";</li>
    <li>STL formula is constructed in "cons_STL.m";</li>
    <li>Optimization problem is solved inside "local_soln.m";</li>
</ul>

Requirements:
<ul type="square">
<!-- li><code>todo</code> </li -->
    <li>Gurobi Optimization Software;</li>
    <li>Matlab Optimization Toolbox (optional to calculate low level MPC trajectories);</li>
</ul>

Paper:

Ali Tevfik Buyukkocak, Derya Aksaray, and Yasin Yazicioglu. "Distributed Planning of Multi-Agent Systems with Coupled Temporal Logic Specifications." AIAA Scitech. 2021, pp. 1123.

