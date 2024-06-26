import planning_playground.motion_models.abstract_motion_model as abstract_motion_model


class KinematicUnicycle(abstract_motion_model.AbstractMotionModel):
    def __init__(self, map):
        super().__init__(map)
