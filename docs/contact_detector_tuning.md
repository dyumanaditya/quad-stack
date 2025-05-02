The tuning of the contact detector include five steps:

1. When we collect the dataset for the localization task, we also collect data with opti-track for the trunk position of the robot.
2. We then use the collected opti-track data to compute the feet positions of the robot.
3. We measured the feet positions in the lab as threshold to get the ground truth of the contact states.
4. Then we perform the localization task and tune the contact detector to be as close as to the ground truth contact states.
