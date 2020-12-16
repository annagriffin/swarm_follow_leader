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
        lock_range=True,
        terms=[
            # Ramp is defined so that start is the bottom of ramp and end is the top
            fl.Ramp('near', 2.5, 0), 
            fl.Triangle('medium', 1.25, 2.5, 3.75),
            fl.Ramp('far', 2.5, 5)
        ]
    ),
    fl.InputVariable(
        name='Right_Laser',
        description='',
        enabled=True,
        minimum=0.0,
        maximum=5.0,
        lock_range=True,
        terms=[
            fl.Ramp('near', 2.5, 0),
            fl.Triangle('medium', 1.25, 2.5, 3.75),
            fl.Ramp('far', 2.5, 5)
        ]
    ),
    fl.InputVariable(
        name='Front_Laser',
        description='',
        enabled=True,
        minimum=0.0,
        maximum=5.0,
        lock_range=True,
        terms=[
            fl.Ramp('near', 2.5, 0),
            fl.Triangle('medium', 1.25, 2.5, 3.75),
            fl.Ramp('far', 2.5, 5)
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
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(), # Maybe play with the resolution of centroid?
        terms=[
            fl.Ramp('very_left', -45, -90),
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
        lock_range=True,
        aggregation=fl.Maximum(),
        defuzzifier=fl.Centroid(), # Maybe play with the resolution of centroid?
        terms=[
            fl.Ramp('reverse', 0, -.5),
            fl.Triangle('slow', -.05, .3, .6),
            fl.Triangle('normal', .3, .6, 1), 
            fl.Ramp('fast', .6, 1)
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
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_Laser is medium \
                            then Rotation is straight_ahead and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is near and Front_Laser is far \
                            then Rotation is straight_ahead and Velocity is fast", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_Laser is medium \
                            then Rotation is right and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is medium and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_Laser is medium \
                            then Rotation is very_right and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is near and Right_Laser is far and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            # Left laser medium
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_Laser is near \
                            then Rotation is very_left and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_Laser is medium \
                            then Rotation is very_left and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is near and Front_Laser is far \
                            then Rotation is very_left and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_Laser is medium \
                            then Rotation is right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is medium and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_Laser is medium \
                            then Rotation is very_right and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is medium and Right_Laser is far and Front_Laser is far \
                            then Rotation is very_right and Velocity is normal", avoidance_engine),
            # Left laser far
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_Laser is near \
                            then Rotation is very_left and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_Laser is medium \
                            then Rotation is very_left and Velocity is slow", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is near and Front_Laser is far \
                            then Rotation is very_left and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_Laser is near \
                            then Rotation is very_left and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_Laser is medium \
                            then Rotation is left and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is medium and Front_Laser is far \
                            then Rotation is left and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_Laser is near \
                            then Rotation is very_right and Velocity is reverse", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_Laser is medium \
                            then Rotation is right and Velocity is normal", avoidance_engine),
            fl.Rule.create("if Left_Laser is far and Right_Laser is far and Front_Laser is far \
                            then Rotation is straight_ahead and Velocity is normal", avoidance_engine),                                                                                                                                                                                                                                                                                                                                                                                             
        ]
    )    
]

if __name__ == "__main__":
    # Testing out the engine
    left_laser = avoidance_engine.input_variable('Left_Laser')
    right_laser = avoidance_engine.input_variable('Right_Laser')
    front_laser = avoidance_engine.input_variable('Front_Laser')

    rotation = avoidance_engine.output_variable('Rotation')
    velocity = avoidance_engine.output_variable('Velocity')

    # print(rotation.terms[0].direction)

    # I have fked up defining linear
    test = velocity.highest_membership(-.4)
    print(test)
    print(test[1].name)
    # print(left_laser.terms)

    # left_laser.value = 3.8
    # print(left_laser.terms)
    # right_laser.value(5)
    # front_laser.value(5)

    test = rotation.highest_membership(10)
    # print(test[0])
    # print(test[1].name)

    # avoidance_engine.process()