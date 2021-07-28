## Benchmark results
### Lingua Franca
#### TimeLimitDistributedDecentralized.lf
A simple benchmark with a source reactor called Clock that periodically (with a period of 1us) sends an integer (that is incremented each time) to a Destination reactor. The benchmark will end by federates calling stop after 10 seconds has passed in logical time. At the end, the elapsed physical time is divided by the number of messages received to get an approximated time spent per reaction.
 - Approx. time per reaction: 5428n
#### TimeLimitDistributedDecentralizedClockSync.lf
This benchmark is a variant of `TimeLimitDistributedDecentralized.lf` that enables clock synchronization.
 - Approx. time per reaction: 5511ns
#### TimeLimitDistributedPhysical.lf
This benchmark is a variant of `TimeLimitDistributedDecentralized.lf` that uses physical connections between Clock and Destination.
 - Approx. time per reaction: 3132ns
#### TimeLimitROS.lf
This benchmark uses ROS to communicate between Clock and Destination (non-federated, not threaded)
 - 100 usec period spin timer: Approx. time per reaction: 43654997ns Dropped messages: 570924
 - 10 usec period spin timer: Approx. time per reaction: 43446746ns Dropped messages: 573300
 - 1 usec period spin timer: Approx. time per reaction: 44510507ns Dropped messages: 564984
### ROS
#### TimeLimitROSNative
The TimeLimit benchmark (see [TimeLimitDistributedDecentralized.lf](#TimeLimitDistributedDecentralized.lf)) implemented in pure ROS. There is a 10 second timeout in physical time for both the Clock and Dest nodes. An approximated time per message received is printed when the benchmark ends.
 - Approx. time per message: 66884ns Dropped messages: 0
