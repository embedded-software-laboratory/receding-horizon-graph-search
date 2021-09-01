# Receding Horizon Graph Search
<!-- icons from https://simpleicons.org/ -->
<!-- [![Paper](https://img.shields.io/badge/-Paper-00629B?logo=IEEE)]()  -->
[![Repository](https://img.shields.io/badge/-GitHub-181717?logo=GitHub)](https://github.com/embedded-software-laboratory/receding-horizon-graph-search) 
[![Video](https://img.shields.io/badge/-Video-FF0000?logo=YouTube)](https://www.youtube.com/watch?v=7LB7I5SOpQE) 
[![Open in Code Ocean](https://codeocean.com/codeocean-assets/badge/open-in-code-ocean.svg)](https://codeocean.com/capsule/7778016/tree/v2)

<!-- GIF: ffmpeg -y -i video_3-circle_RHC.mp4 -vf "crop=in_h:in_h:420:0,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 preview.gif -->
<a href="https://gifyu.com/image/GGVg"><img src="https://s9.gifyu.com/images/preview6ec7fc3f8852dd89.gif" alt="preview6ec7fc3f8852dd89.gif" border="0" width="320"/></a>

This repository contains the source code for Receding Horizon Graph Search (RHGS), a MATLAB implementation of a graph-based receding horizon trajectory planner.

The code is developed with MATLAB R2021a.
To run a simulation:
```matlab
startup()
main()
```

More information is provided in our publication [1], which we kindly ask you to consider citing if you find RHGS helpful for your work.
The results of the publication can be reproduced by running
```matlab
startup()
eval_paper()
```
This will take a while. The results are then found in the folder "results".
### Acknowledgements
This research is supported by the Deutsche Forschungsgemeinschaft (German Research Foundation) within the Priority Program SPP 1835 "Cooperative Interacting Automobiles" (grant number: KO 1430/17-1).

### References

<details>
<summary>
[1] P. Scheffe, M. V. A. Pedrosa, K. Flaßkamp and B. Alrifaee,
"Receding Horizon Control Using Graph Search for Multi-Agent Trajectory Planning"
</summary>
<p>
```
bibtex
```
</p>
</details>