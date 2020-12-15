""" Code establishing the fuzzy engine """

import fuzzylite as fl

avoidance_engine = fl.Engine(
    name='collision_avoidance',
    description=''
)

avoidance_engine.input_variables = [
    fl.InputVariable(
        name='Left_Laser',
        description='',
        enabled=True,
        minimum=0.0, # The true range_min specified in sensor_msgs/LaserScan is ~0.1 
        maximum=5.0,
        lock_range=False,
        terms=[
            fl.Linear('near', [-.4, 1]),
            fl.Triangle('medium', 1.25, 2.5, 3.75),
            fl.Linear('far', [.4, -1])
        ]
    ),
    fl.InputVariable(
        name='Right_Laser',
        description='',
        enabled=True,
        minimum=0.0,
        maximum=5.0,
        lock_range=False,
        terms=[
            fl.Linear('near', [-.4, 1]),
            fl.Triangle('medium', 1.25, 2.5, 3.75),
            fl.Linear('far', [.4, -1])
        ]
    ),
    fl.InputVariable(
        name='Front_Laser',
        description='',
        enabled=True,
        minimum=0.0,
        maximum=5.0,
        lock_range=False,
        terms=[
            fl.Linear('near', [-.4, 1]),
            fl.Triangle('medium', 1.25, 2.5, 3.75),
            fl.Linear('far', [.4, -1])
        ]
    ),                        
]

avoidance_engine.output_variables = [
    fl.OutputVariable(
        name='Rotation',
        description='',
        enabled=True,
        minimum=-90,
        maximum=90,
        lock_range=False,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(), # Maybe play with the resolution of centroid?
        terms=[
            fl.Ramp('very_left', -90, -45),
            fl.Triangle('left', -90, -45, 0),
            fl.Triangle('straight_ahead', -45, 0, 45),
            fl.Triangle('right', 0, 45, 90),
            fl.Ramp('very_right', 45, 90)
        ]
    ),
    fl.OutputVariable(
        name='Velocity',
        description='',
        enabled=True,
        minimum=-.5,
        maximum=1,
        lock_range=False,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(), # Maybe play with the resolution of centroid?
        terms=[
            fl.Linear('reverse', [-2, 0]),
            fl.Triangle('slow', 0, .3, .6),
            fl.Triangle('normal', .3, .6, 1), 
            fl.Linear('fast', [2.5, -1.5])
        ]
    ),
]

avoidance_engine.rule_blocks = [
    fl.RuleBlock(
        name="mamdani",
        description="",
        enabled=True,
        conjunction=fl.Minimum(),
        disjunction=fl.Maximum(),
        implication=fl.Minimum(),
        activation=fl.General(),
        rules=[
            # Left laser near section
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_laser is medium \
                            then Rotation is straight_ahead and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_laser is far \
                            then Rotation is straight_ahead and Velocity is fast", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_laser is medium \
                            then Rotation is right and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_laser is medium \
                            then Rotation is very_right and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            # Left laser medium
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_laser is near \
                            then Rotation is very_left and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_laser is medium \
                            then Rotation is very_left and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_laser is far \
                            then Rotation is very_left and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_laser is medium \
                            then Rotation is right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_laser is medium \
                            then Rotation is very_right and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            # Left laser far
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_laser is near \
                            then Rotation is very_left and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_laser is medium \
                            then Rotation is very_left and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_laser is far \
                            then Rotation is very_left and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_laser is near \
                            then Rotation is very_left and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_laser is medium \
                            then Rotation is left and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_laser is far \
                            then Rotation is left and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_laser is medium \
                            then Rotation is right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_laser is far \
                            then Rotation is straight_ahead and Velocity is normal", avoidance_engine),                                                                                                                                                                                                                                                                                                                                                                                             
            fl.Rule.create("if Load is small and Dirt is not high then Detergent is less_than_usual", engine),
        ]
    )    
]

