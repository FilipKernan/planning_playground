import os

# See if Cython is installed
try:
    from Cython.Build import cythonize
    from setuptools import Extension, setup
# Do nothing if Cython is not available
except ImportError:
    # Got to provide this function. Otherwise, poetry will fail
    def build(setup_kwargs):
        pass
# Cython is installed. Compile
else:
    from distutils.command.build_ext import build_ext

    # This function will be executed in setup.py:
    def build(setup_kwargs):
        # The file you want to compile
        extensions = [
            Extension(
                "planning_playground",
                [
                    "planning_playground/map/abstract_map.py",
                    "planning_playground/map/import_map.py",
                    "planning_playground/motion_models/abstract_motion_model.py",
                    "planning_playground/motion_models/holonomic_model.py",
                    "planning_playground/motion_models/kinematic_model.py",
                    "planning_playground/motion_models/kinematic_bicycle.py",
                    "planning_playground/motion_models/kinematic_unicycle.py",
                    "planning_playground/search/abstract_planner.py",
                    "planning_playground/search/rrt_planner.py",
                    "planning_playground/search/rrt_star_planner.py",
                    "planning_playground/search/types.py",
                    "planning_playground/smoothers/spline_smoother.py",
                    "planning_playground/smoothers/interp_smoother.py",
                    "planning_playground/viz/viz_plan.py",
                ],
            )
        ]

        # gcc arguments hack: enable optimizations
        os.environ["CFLAGS"] = "-O3"

        # Build
        setup_kwargs.update(
            {
                "ext_modules": cythonize(
                    extensions,
                    build_dir="build",
                    language_level=3,
                    compiler_directives={"linetrace": True},
                ),
                "cmdclass": {"build_ext": build_ext},
            }
        )
