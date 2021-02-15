from multitaxienv.taxi_environment import TaxiEnv, orig_MAP, MAP2, orig_MAP2, MAP3
from TaxiWrapper.taxi_wrapper import *
from ControllerWrapper.controller_wrapper import Controller
import matplotlib.pyplot as plt
import copy


def no_collaboration_case(taxi_env: TaxiEnv, controller: Controller, taxis: List[Taxi], passenger_index: int):
    """
    Check if the taxis are able to bring the passenger (given by the passenger_index) to her destination if they are not
    collaborating.
    Return:
        - A list with the indices of the taxis that are able to pick up the passenger and bring her to the destination.
        - The minimum distance that the taxis can bring the passenger from the destination (0 if arriving a the dest).
    """
    passenger_location = taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
    passenger_destination = taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]
    path_to_destination_cost = controller.env_graph.path_cost(origin=passenger_location, dest=passenger_destination)
    capable_taxis = []  # list with the indices of the taxis that can bring the passenger to the destination.
    min_dist_from_dest = path_to_destination_cost  # the minimum distance that the passenger can get from the dest.
    for taxi in taxis:
        path_to_passenger_cost = taxi.path_cost(dest=passenger_location)
        total_path_cost = path_to_passenger_cost + path_to_destination_cost
        taxi_fuel = taxi.get_fuel()
        dist_from_dest = max(0, total_path_cost - (taxi_fuel - 1))
        min_dist_from_dest = min(min_dist_from_dest, dist_from_dest)
        if total_path_cost < taxi_fuel:
            capable_taxis.append(taxi.taxi_index)
    return capable_taxis, min_dist_from_dest


def collaboration_case(taxi_env: TaxiEnv, controller: Controller, taxis: List[Taxi], passenger_index: int, h: int):
    """
    Check if the taxis are able to bring the passenger (given by the passenger_index) to the destination,
    when collaborating according to a given heuristic. The possible heuristics are:
        - 0: optimal (exhaustive search)
        - 1: heuristic 1
        - 2: heuristic 2
        see `controller_wrapper/README.md` for more details.
    Return:
        - 1 if the taxis are able to bring the passenger to the destination, 0 otherwise.
        - The remaining distance of the passenger from her destination (0 if the passenger arrived at the destination).
    """
    passenger_location = controller.get_passenger_cors(passenger_index)

    # Allocate the passenger to the closest taxi:
    closest_taxi = controller.find_closest_taxi(dest=passenger_location)
    taxis[closest_taxi].assigned_passengers.append(passenger_index)
    if closest_taxi == -1:  # No taxi has enough fuel to pickup the passenger.
        passenger_destination = controller.get_destination_cors(passenger_index)
        remaining_dist_to_dest = controller.env_graph.path_cost(origin=passenger_location, dest=passenger_destination)
        return 0, remaining_dist_to_dest

    # Send the taxi to pick up the passenger:
    taxis[closest_taxi].send_taxi_to_pickup()
    controller.execute_all_actions()

    # Compute the transfer point according to different heuristics and execute the transfer:
    to_taxi_index = 1 - closest_taxi
    if h == 1:
        transfer_point = controller.find_transfer_point_h1(from_taxi_index=closest_taxi, passenger_index=0,
                                                           to_taxi_index=to_taxi_index)
    elif h == 2:
        transfer_point = controller.find_transfer_point_h2(from_taxi_index=closest_taxi, passenger_index=0)
    else:
        transfer_point = controller.find_optimal_transfer_point(from_taxi_index=closest_taxi,
                                                                to_taxi_index=to_taxi_index,
                                                                passenger_index=passenger_index)
    controller.transfer_passenger(passenger_index=0, from_taxi_index=closest_taxi, to_taxi_index=to_taxi_index,
                                  transfer_point=transfer_point)

    # Send the second taxi to dropoff the passenger at her destination:
    to_taxi = to_taxi_index
    controller.taxis[to_taxi].send_taxi_to_dropoff()
    controller.execute_all_actions()
    if controller.taxi_env.state[PASSENGERS_STATUS][0] == 1:  # True if passenger arrived at destination,
        return 1, 0
    else:
        passenger_location = controller.get_passenger_cors(passenger_index)
        passenger_destination = controller.get_destination_cors(passenger_index)
        remaining_dist_to_dest = controller.env_graph.path_cost(origin=passenger_location, dest=passenger_destination)
        return 0, remaining_dist_to_dest


def reset_env_state(state, env, controller, all_taxis):
    """
    Reset the environment state to the given state. The reset is done to the each environment held by the controller
    and taxis.
    """
    env.state = state
    controller.taxi_env.state = state
    for taxi in all_taxis:
        taxi.taxi_env.state = state
    controller.taxis = all_taxis


def collaboration_experiment(test_repetitions: int, num_taxis: int, taxis_fuel: List[int]):
    """
    This experiment compares the number of successful passenger deliveries and the distance of the passenger from the
    destination when not using taxi-collaboration and when collaboration according to one of the 3 different
    heuristics we examined.
    Args:
        test_repetitions: the number of times to repeat the test. For every test, a new random environment is
        initialized.
        num_taxis: the number of taxis that should be initialized in every test.
        taxis_fuel: a list of size `num_taxis`, where each element is the maximal fuel value for every taxi.
    """
    no_collaboration_success = 0
    average_dist_no_collaboration = 0
    collaboration_h1_success = 0
    average_dist_h1_collaboration = 0
    collaboration_h2_success = 0
    average_dist_h2_collaboration = 0
    collaboration_optimal_success = 0
    average_dist_optimal_collaboration = 0

    for test in range(test_repetitions):
        env = TaxiEnv(num_taxis=num_taxis, num_passengers=1, max_fuel=taxis_fuel,
                      taxis_capacity=None, collision_sensitive_domain=False,
                      fuel_type_list=None, option_to_stand_by=True, domain_map=MAP3)
        env.reset()
        env.s = 1022

        # Initialize a Taxi object for each taxi and a controller:
        all_taxis = []
        for i in range(num_taxis):
            all_taxis.append(Taxi(env, taxi_index=i))
        controller = Controller(env, taxis=all_taxis)

        no_collaboration_results = no_collaboration_case(env, controller, all_taxis, passenger_index=0)
        if no_collaboration_results[
            0]:  # if the list is not empty, there is a taxi capable of taking the pass to the dest.
            no_collaboration_success += 1
            collaboration_h1_success += 1
            collaboration_h2_success += 1
            collaboration_optimal_success += 1
        else:
            # Add the distance of the passenger to the no_collaboration_average as the passenger didn't arrive at dest.
            average_dist_no_collaboration += no_collaboration_results[1] / test_repetitions

            # copy the env state so the same state can be used for all heuristics:
            state = copy.deepcopy(env.state)

            # test heuristic 1
            collaboration_h1_results = collaboration_case(env, controller, all_taxis, passenger_index=0, h=1)
            collaboration_h1_success += collaboration_h1_results[0]
            average_dist_h1_collaboration += collaboration_h1_results[1] / test_repetitions

            # reset the env to the state before the collaboration test:
            reset_env_state(state, env, controller, all_taxis)
            state = copy.deepcopy(env.state)

            # test heuristic 2
            collaboration_h2_results = collaboration_case(env, controller, all_taxis, passenger_index=0, h=2)
            collaboration_h2_success += collaboration_h2_results[0]
            average_dist_h2_collaboration += collaboration_h2_results[1] / test_repetitions

            # reset the env to the state before the collaboration test:
            reset_env_state(state, env, controller, all_taxis)
            state = copy.deepcopy(env.state)

            # test the optimal solution
            collaboration_optimal_results = collaboration_case(env, controller, all_taxis, passenger_index=0, h=0)
            collaboration_optimal_success += collaboration_optimal_results[0]
            average_dist_optimal_collaboration += collaboration_optimal_results[1] / test_repetitions

    return no_collaboration_success, average_dist_no_collaboration, collaboration_h1_success, \
           average_dist_h1_collaboration, collaboration_h2_success, average_dist_h2_collaboration, \
           collaboration_optimal_success, average_dist_optimal_collaboration


def collaboration_statistics(test_repetitions: int):
    """
    Repeat the collaboration_experiment multiple times with different fuel values for the taxis, and measure the number
    of successes under the different fuel limits, both in the collaborative and non-collaborative settings.
    """
    fuel_limits = range(3, 17)
    no_collaboration_successes = []
    no_collaboration_dist = []
    collaboration_h1_successes = []
    collaboration_h1_dist = []
    collaboration_h2_successes = []
    collaboration_h2_dist = []
    collaboration_optimal_successes = []
    collaboration_optimal_dist = []
    for fuel in fuel_limits:
        results = collaboration_experiment(test_repetitions=test_repetitions, num_taxis=2, taxis_fuel=[fuel, fuel])
        no_collaboration_successes.append(results[0] / test_repetitions * 100)
        no_collaboration_dist.append(results[1])
        collaboration_h1_successes.append(results[2] / test_repetitions * 100)
        collaboration_h1_dist.append(results[3])
        collaboration_h2_successes.append(results[4] / test_repetitions * 100)
        collaboration_h2_dist.append(results[5])
        collaboration_optimal_successes.append(results[6] / test_repetitions * 100)
        collaboration_optimal_dist.append(results[7])

    # Plot success rate graph:
    plt.plot(fuel_limits, no_collaboration_successes, 'b-', fuel_limits, collaboration_h1_successes, 'r-', fuel_limits,
             collaboration_h2_successes, 'g--', fuel_limits, collaboration_optimal_successes, 'k--')
    plt.xlabel('Fuel Level')
    plt.ylabel('% of times passenger arrived at destination')
    plt.suptitle('Collaboration vs. Non-Collaboration')
    plt.legend(('No Collaboration', 'With Collaboration H1', 'With Collaboration H2', 'Optimal'), loc='upper left')
    plt.show()

    # Plot distance from destination graph:
    plt.plot(fuel_limits, no_collaboration_dist, 'b-', fuel_limits, collaboration_h1_dist, 'r-', fuel_limits,
             collaboration_h2_dist, 'g--', fuel_limits, collaboration_optimal_dist, 'k--')
    plt.xlabel('Fuel Level')
    plt.ylabel('Average distance from destination.')
    plt.suptitle('Collaboration vs. Non-Collaboration')
    plt.legend(('No Collaboration', 'With Collaboration H1', 'With Collaboration H2', 'Optimal'), loc='upper right')
    plt.show()


if __name__ == '__main__':
    collaboration_statistics(test_repetitions=500)
