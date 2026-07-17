# Robot Subsystems

`engine.robot.arm` owns arm kinematics, motor mapping, sag, and mounts.
`engine.robot.go2` owns hardware bridging, locomotion, and MPC. Neither
package owns UI state or end-to-end Pick/Gaze workflow decisions.
