Type of controller: Feedforward & P
Kp = 1.5

If you are using PI control and find that the error twist displays a non-converging, chaotic pattern, it might be the internal computational error from matlab. If adjusting the gains does not solve the problem, uncomment the disp in FeedbackControl.m and try again.