## Multi-Taxi Collaboration
In this project we aim to show that collaboration between taxis can be beneficial both for the taxi drivers and
 passengers. In multiple settings and under fuel limits we show that the success-rate of passengers arrival at their
  destination increases significantly when taxis are collaborating and can transfer a passenger from
   one taxi to the other. Moreover, we showed that even if the passenger can't be brought to the destination, the
    average distance of the passenger from the destination is decreased.
    
## Implementation
Our implementation allows the user to run different experiments that demonstrate the usefulness of collaboration both
 in a centralized and decentralized control settings.
 
### Taxi Wrapper
A wrapper for a single taxi object. Allows to represent and control a single taxi and move it in the environment.
The taxi wrapper includes two classes:
1. **EnvGraph Class**: This class converts the original string representation of the grid to a graph representation of
 the *Networkx* library. Using the graph representation, multiple computations and path calculations can be performed
  such as shortest path between two points, etc.
  When initializing an EnvGraph object, the map that is converted to a graph is the map with which the original TaxiEnv
   object was initialized.
2. **Taxi Class**: This class wraps a single taxi. The class can be used to control a single taxi object, and
 includes functions allowing to send the taxi to pickup a certain passenger from a given location and dropoff a
  passenger at a given point. Moreover, it includes function that allow multiple taxis to communicate using a
   messaging system. 

When initializing a Taxi object the following arguments should be passed:
1. The current environment.
2. The index of the taxi this object represents.
3. (optional) The list of passengers indices that this taxi is responsible of.

##### Taxi Wrapper Demo
The `taxi_wrapper_demo.py` includes a simple example how to use the taxi class.

### Controller Wrapper
The controller wrapper implements the `Controller` class which includes functions that allow the user to control the
 taxis in a centralized manner. The class includes functions that compute the transfer point of a passenger between
  taxis. The transfer point can be computed according to the following 3 options:
  1. Optimal transfer-point: check all points on the grid and select the point that minimizes the distance of the
   passenger from the destination after the transfer.
  2. Heuristic 1: bring the passenger as close as possible to the path of the second taxi to the destination (the taxi
   that should deliver the passenger to the destination).
  3. Heuristic 2: bring the passenger as far as possible on her shortest path to the destination. This heuristic
   doesn't depend on the location of the second taxi.

   When initializing a Controller object the following arguments should be passed:
   1. The current environment .
   2. A list with all taxis that the controller should control.

##### Controller Wrapper Demo
The `controller_wrapper_demo.py` includes a simple example how to use the controller class to control the taxis and
 make them collaborate in a centralized manner. 

## Centralized & Decentralized Control
As mentioned above, the controller wrapper allows the user to control the taxis and make them collaborate in a
 centralized manner. In the `Decentralized_Control_Example.py` file we show how the collaboration between taxis can
  be done in a decentralized manner, i.e. taxis communicate with each other in order to decide which taxis should
   pickup the passenger and deliver her to the destination, and where the transfer should happen.

## Collaboration Experiment
In the `Collaboration_Experiment.py` file we demonstrate the significance of collaboration. In this experiment we
 compare the percentage of successful passenger deliveries to the destination and the distance of the passenger from
  the destination under 4 different settings:
  1. No collaboration - taxis are not allowed to collaborate with each other.
  2. Collaborating using a naive optimal solution. 
  3. Collaborating using heuristic 1. 
  4. Collaborating using heuristic 2.
  See the [Controller Wrapper](#controller-wrapper) section for more information about the different heuristics.
  
### Results
The graphs below show the different results achieved when using the 4 different methods mentioned above.

Success Rate             |  Distance From Destination
:-------------------------:|:-------------------------:
![](https://user-images.githubusercontent.com/61732335/107937905-43976800-6f8d-11eb-9bf6-f50d25175990.png)  |  ![](https://user-images.githubusercontent.com/61732335/107937910-46925880-6f8d-11eb-9038-ff56ea6865e6.png)

On the left graph, we can see that when taxis are allowed to collaborate, the success-rate of passenger deliveries is
 significantly higher under different fuel limitations. 
 On the right graph, we can see that the average distance of the passenger from the destination is smaller when using
  taxi collaboration.
 


