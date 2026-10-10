# Quiz 6 · Faults and fault tolerance

*10 minutes, individual. Answers: `instructor/quiz-keys.md`.*

1. A rotor has 70 % loss of effectiveness. What is its $\eta$?

2. A quad has a thrust-to-weight ratio of 2.5. Below which $\eta$ of a single rotor is hover
   with the heading held impossible? Show the one-line argument.

3. What is a fault signature, and how does projecting the residual on the signatures
   isolate the faulty rotor?

4. In a CUSUM test, what happens to the detection delay and to the false-alarm rate when
   you double the threshold $h$?

5. Why does the rotorfault study use a *bank* of CUSUM tests (15, 30, 60 % LOE) rather than one?

6. Rotor 1 is estimated at $\hat\eta_1 = 0.5$, the others healthy. In hover, how much more
   must rotor 1 be commanded than before the fault, according to the re-allocation
   $f = (B\,\mathrm{diag}(\hat\eta))^{-1}[T, \tau]$?

7. Why must a quadrotor that loses a rotor completely give up control of its heading, and
   what does it control instead?

8. Name two limitations stated by the rotorfault study that would most change its results
   on a real 5-inch quad, and say how you would measure each.
