
class Controller:
    def __init__(self, taxi_env, taxis):
        self.taxi_env = taxi_env
        self.taxis = taxis

    def get_next_step(self):
        # Check that not all taxis completed all steps:
        if self.not_all_taxis_completed_path():
            taxis_steps = []
            for taxi in self.taxis:
                step = taxi.get_next_step()
                if step:
                    taxis_steps.append(step[1])
                else:  # if there are no actions left in the path of the taxi, stay in place.
                    taxis_steps.append(self.taxi_env.action_index_dictionary['standby'])

    def set_meeting_point(self, point):
        """
        Compute the path of all taxis to a given meeting point.
        """
        for taxi in self.taxis:
            taxi.compute_shortest_path(point)

    def not_all_taxis_completed_path(self):
        """
        Check if not all taxis completed their paths.
        Return `True` if not all taxis completed their path and `False` if some taxi still has steps to do.
        """
        taxi_not_completed_path = [taxi for taxi in self.taxis if taxi.path_cords]
        return any(taxi_not_completed_path)


