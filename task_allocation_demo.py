import numpy as np

from TaxiWrapper.taxi_wrapper import *
from TaxiWrapper.BiddingTaxi import BiddingTaxi
from multitaxienv.taxi_environment import TaxiEnv
from TaskAllocator import TaskAllocator

def create_taxis_list(env, taxis_num):
    taxis_list = []
    for i in range (taxis_num):
        wrapped_taxi = BiddingTaxi(env, taxi_index=i)
        taxis_list.append(wrapped_taxi)
    
    return taxis_list


def run_simulation(env, taxis_num):

    env.reset()
    # env.s = 1022
    env.render()

    allocator = TaskAllocator(env)

    taxis_list = create_taxis_list(env, taxis_num)

    taxis_biddings = allocator.get_taxis_bids(taxis_list)
    biddings_allocation = allocator.taxis_auction_allocation(taxis_biddings)
    biddings_allocation_cost = allocator.allocation_cost(biddings_allocation)
    print(biddings_allocation)
    print(biddings_allocation_cost)

    print("---------")
    
    true_allocations_cost = allocator.passengers_allocations_cost()
    optimal_allocation = allocator.optimal_allocation_minimal_value(true_allocations_cost)
    # optimal_allocation_dict = {taxi_ind:passenger_ind for (taxi_ind,passenger_ind) in optimal_allocation}
    optimal_allocation_cost = allocator.allocation_cost(optimal_allocation)
    print("---------")
    print(optimal_allocation)
    print(optimal_allocation_cost)

    return biddings_allocation_cost, optimal_allocation_cost

def main():
    

    # Assuming number of taxis is equal to the number of passensgers
    taxis_num = passengers_num = 4

    env = TaxiEnv(num_taxis=taxis_num, num_passengers=passengers_num, max_fuel=None,
                    taxis_capacity=None, collision_sensitive_domain=False,
                    fuel_type_list=None, option_to_stand_by=True)
    
    env.reset()
    env.s = 1022
    env.render()

    allocator = TaskAllocator(env)

    taxis_list = create_taxis_list(env, taxis_num)

    taxis_biddings = allocator.get_taxis_bids(taxis_list)
    auction_allocation = allocator.taxis_auction_allocation(taxis_biddings)
    auction_allocation_cost = allocator.allocation_cost(auction_allocation)
    print("Auction based allocation is: {auction_allocation}, cost is {auction_allocation_cost}".format(auction_allocation=auction_allocation,
                                                                        auction_allocation_cost=auction_allocation_cost))

    print("---------")
    
    true_allocations_cost = allocator.passengers_allocations_cost()
    optimal_allocation = allocator.optimal_allocation_minimal_value(true_allocations_cost)
    optimal_allocation_cost = allocator.allocation_cost(optimal_allocation)
    print("Optimal allocation is: {optimal_allocation}, cost is {optimal_allocation_cost}".format(optimal_allocation=optimal_allocation,
                                                                        optimal_allocation_cost=optimal_allocation_cost))

    TaskAllocator.allocate_passengers(optimal_allocation, taxis_list)

    # From here, one can start moving the taxi, e.g.
    # in a straight-forward way as shown in taxi_wrapper_demo.py
    
    


if __name__ == "__main__":
    main()