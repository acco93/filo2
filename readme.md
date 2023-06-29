#### FILO2
FILO2 is an evolution of [FILO](https://github.com/acco93/filo) able to tackle much larger CVRP instances with a very ordinary computing system. 
Less than ten minutes and 10GB of RAM are enough to execute 100k FILO2 iterations and get a (possibly very bad) solution for an instance with 1 million customers. 
On the [X](instances/X/) and [B](instances/B/) datasets FILO2 behaves pretty much like FILO in terms of quality (sometimes it is even slightly better) but it has a considerably lower time growth rate that becomes very visible for instances with more than 5k customers.

Changes and computational results are described in [this draft](https://arxiv.org/abs/2306.14205). This repository contains [raw results](results/) and will soon contain the algorithm source code.
