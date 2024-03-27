Motion library can be benefited with the help of LLM by carrying out promt or description based input to the LLMs.

LLM can act as an interface between the user and the Python API which is based on the motion-library. The communication interface for this can be given as -


<img src="https://github.com/mrunmaivp/ur_manipulation_repo/blob/main/Picture1.png" alt="High Level Architecture" width="600" height="300">

Architecture similar to the above image can be used to generate a prompt or description about the motion to be carried out. Based on the motion input,
the LLM can be used to generate or modify the existing code for motion planning in joint or cartesian space. Code Suggestions provided by the LLM can be selected by
the user which aligns better with the task.

