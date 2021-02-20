taxi_env_rewards = dict(
    step=0,
    no_fuel=0,
    bad_pickup=0,
    bad_dropoff=0,
    bad_refuel=0,
    pickup=0,
    standby_engine_off=0,
    turn_engine_on=0,
    turn_engine_off=0,
    standby_engine_on=0,
    intermediate_dropoff=0,
    final_dropoff=1,
    hit_wall=-100,
    collision=0,
    passenger_killed=0,
)

all_action_names = ['south', 'north', 'east', 'west',
                    'pickup', 'dropoff',
                    'turn_engine_on', 'turn_engine_off',
                    'standby',
                    'refuel']

base_available_actions = ['south', 'north', 'east', 'west',
                          'pickup', 'dropoff']
