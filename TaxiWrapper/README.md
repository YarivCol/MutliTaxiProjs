##**Taxi Wrapper**
The taxi wrapper includes two classes:
1. **EnvGraph Class**: This class converts the original string representation of the grid to a graph representation of
 the *Networkx* library. Using the graph representation, multiple computations and path calculations can be performed
  such as shortest path between two points, etc.
  When initializing an EnvGraph object, the map that is converted to a graph is the map with which the original TaxiEnv
   object was initialized.
2. **Taxi Class**: This class wraps a single taxi. The class can be used to compute the path of the taxi to a specific
 point (currently supports only shortest path computation from the current position of the taxi to a given
  destination point using the `compute_shortest_path` function) and to get the next step that should be taken
   according to the calculated path, using the `get_next_step` function.
   
   When initializing a Taxi object the following arguments should be passed:
   1. The current environment state.
   2. The index of the taxi this object represents.
   3. (optional) The index of the passenger that this taxi is responsible of.
   
####Taxi Wrapper Demo
The `taxi_wrapper_demo.py` file was added as an example how to use the Taxi object.
This demo includes the following steps:
1. Initialize a new environment with 1 taxi and 2 passengers.
2. Initialize a new taxi object for the taxi, compute the shortest path from its current location to the
 position of the second passenger.
3. Pickup the second passenger.
4. Compute the shortest path to the passenger's destination and drive there.
5. Drop off the passenger at the destination.
   
**Note:**
The `config.py` and `taxi_environment.py` files in this folder are the original files from the *MultiTaxiEnv* project. 