import planning_playground.planners.abstract_planner as abstract_planner


class RRTStarPlanner(abstract_planner.AbstractPlanner):
    def __init__(self, map, motion_model):
        super().__init__(map, motion_model)
        self.nodes = {}
        self.start_node = None
        self.goal = None
        self.goal_node = None
