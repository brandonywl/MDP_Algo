# CZ3004 Multi-Disciplinary Project Pathfinding Algo
For fulfillment of CZ3004, a project in which we have to navigate a robot through a 20x20 board with 5 obstacles. Each obstacle takes up a 1x1 space. On each obstacle, there is only one side where an alphanumeric character is shown.

The car's wheels were controlled via a singular motor and could not provide differential drive. Hence we opted for a kinematic bicycle model to compute the expected location of the vehicle. Reference https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357

The goal of the algorithms in this project is to find the fastest path to all 5 obstacles, while positioning our car in a optimal position to take a photo of the character. The following algorithms are tested:
1. Reeds-Shepp Path Planning (referencing http://lavalle.pl/planning/node822.html)
    - Failed to have consistent convergence or required a really long path. Unsure of how to improve on it.
2. Deep Reinforcement Learning with Deep Q Networks
    - Bad convergence, training was inconsistent and often could not find a path.
3. Hybrid A-star algorithm
    - Best choice as it consistently lead to an answer. Each node expanded with a 6 states with a specific angle of turn, forward, back, forward left, forward right, backward left, backward right. 

The programme uses ```pygame``` for visualizations of the path planned. 

The algorithm uses computer vision for corrections of the actual path and the expected path. The image classification model was trained using Yolov5. We noticed that the actual position of the car was consistently off from the expected position in a straight manner, regardless of the previous steps. This is possibly due to the shakiness of the wheels to be used.
This is not an issue for the first few images, but it would result in larger errors at later obstacles. Hence, to prevent the error from propagating further we:
1. Relabelled our data with tighter bounding boxes to ensure consistency of the trained model's output
2. Obtained the average bounding box height and width of images taken at the distance our pathing algorithm aims for.
3. Compute the expected amount of distance to move given the difference in the average bounding boxes of images and the current image's bounding box. We did this mostly using pythagoras theorem, ignoring possible slants and assuming the photo is taken in parallel.

Our robot was able to achieve all 5 obstacles due to this optimization using computer vision. However, 2 points were deducted due to a wrong classification due to bright light.
