# NXP
NXP car repository for Univerisity project

## ToD0 before presentation:

- [ ] Re-check COM port speed/camera/servo operation
- [ ] Adjust KP to be super fast (1.9 was superb)
- [ ] Set target closer to inner edge and higher max speed
- [ ] Turn steepness adjustment to improve at high speeds

## Start protocol:
  1. Servo error correction manually with POT0
  2. Camera at 45/50 angle adjustment as picture on FB
  3. MOTOR-OFF, Print: PID, LINE_POS
  4. Dry lap to see correction with KP gain
  5. Serial OFF
  6. Hot run on constant speed
  5. KP adjust - 4.
  6. Variable speeds
  7. Operational run
